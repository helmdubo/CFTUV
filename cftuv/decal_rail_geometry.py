"""Compile/evaluate split PLANAR rail geometry (TRANCHE R1).

R1 materializes only topology components whose complete rail ownership is
compile-time PLANAR.  The evaluator never traces topology and never chooses a
route: drag only clips immutable channel cells by the current half-width.
Internal coordinates are the neutral rail-domain pair ``(s, r)``.  The
``DecalGeometryFace`` UV arrays are a temporary adapter for the existing mesh
writer; R4 remains the sole owner of the public ``(s, r) -> (U, V)`` mapping.
"""

from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass, replace
from math import acos, pi, sqrt

from mathutils import Vector

from .constants import DECAL_CORNER_MITER_LIMIT
from .decal_geometry import DecalGeometryFace
from .decal_rails import RailStationKind, RailTermination


# Это только arithmetic validity допуск представления одной плоскости, а не
# выбор поведения: любой различимый source-fold остаётся вне PLANAR R1.
_RAIL_PLANAR_DOT = 1.0 - 1e-10


@dataclass(frozen=True)
class RailGeometryFailure:
    reason: str
    edge_indices: tuple[int, ...] = ()
    vertex_indices: tuple[int, ...] = ()
    face_indices: tuple[int, ...] = ()
    route_ids: tuple[int, ...] = ()
    details: tuple[tuple[str, object], ...] = ()


@dataclass(frozen=True)
class RailGeometryCompileAttempt:
    plan: "PlanarRailGeometryPlan | None"
    failures: tuple[RailGeometryFailure, ...] = ()


class RailGeometryEvaluationError(RuntimeError):
    def __init__(self, failure):
        super().__init__(failure.reason)
        self.failure = failure


class _RailGeometryCompileError(RuntimeError):
    def __init__(
        self,
        reason,
        *,
        edge_indices=(),
        vertex_indices=(),
        face_indices=(),
        route_ids=(),
        details=(),
    ):
        super().__init__(str(reason))
        self.failure = RailGeometryFailure(
            reason=str(reason),
            edge_indices=tuple(sorted({int(value) for value in edge_indices})),
            vertex_indices=tuple(
                sorted({int(value) for value in vertex_indices})
            ),
            face_indices=tuple(sorted({int(value) for value in face_indices})),
            route_ids=tuple(sorted({int(value) for value in route_ids})),
            details=tuple(details),
        )


@dataclass(frozen=True)
class RailSpineStation:
    station_index: int
    source_vertex_id: int
    s: float


@dataclass(frozen=True)
class RailSpineInterval:
    interval_index: int
    source_edge_id: int
    from_station_index: int
    to_station_index: int
    source_face_ids: tuple[int, ...]


@dataclass(frozen=True)
class RailSpineComponent:
    component_id: int
    edge_ids: tuple[int, ...]
    vertex_ids: tuple[int, ...]
    stations: tuple[RailSpineStation, ...]
    intervals: tuple[RailSpineInterval, ...]
    is_closed: bool = False


@dataclass(frozen=True)
class RailBoundaryVertex:
    """Compile-static vertex of a topological or virtual rail piece."""

    key: tuple
    position: tuple[float, float, float]
    r: float
    source_feature: str
    source_feature_id: object
    source_edge_id: int | None
    route_id: int
    station_index: int | None


@dataclass(frozen=True)
class RailBoundaryPiece:
    piece_index: int
    owner_face_id: int
    start: RailBoundaryVertex
    end: RailBoundaryVertex
    source_edge_ids: tuple[int, ...]


@dataclass(frozen=True)
class RailCapTrace:
    """RM5a virtual station-0 rail, compiled once in an owner plane."""

    trace_id: int
    route_id: int
    spine_vertex_id: int
    spine_edge_id: int
    initial_face_id: int
    direction: tuple[float, float, float]
    pieces: tuple[RailBoundaryPiece, ...]
    termination: str


@dataclass(frozen=True)
class RailBoundaryPath:
    path_id: tuple
    route_id: int
    spine_vertex_id: int
    kind: str
    pieces: tuple[RailBoundaryPiece, ...]
    termination: str
    cap_trace_id: int | None = None


@dataclass(frozen=True)
class RailDomainVertex:
    key: tuple
    position: tuple[float, float, float]
    s: float
    r: float
    boundary_path_id: tuple
    source_feature: str
    source_feature_id: object
    source_edge_id: int | None
    source_face_id: int
    route_ids: tuple[int, ...]
    station_index: int | None


@dataclass(frozen=True)
class RailChannelCell:
    cell_id: int
    channel_id: int
    owner_face_id: int
    spine_edge_id: int
    source_edge_ids: tuple[int, ...]
    boundary_path_ids: tuple[tuple, tuple]
    vertices: tuple[RailDomainVertex, ...]


@dataclass(frozen=True)
class RailChannel:
    channel_id: int
    spine_edge_id: int
    initial_face_id: int
    side_sign: int
    from_path_id: tuple
    to_path_id: tuple
    cells: tuple[RailChannelCell, ...]
    alpha_limit: float = 0.0


@dataclass(frozen=True)
class RailCornerUse:
    """Stable A10 ownership tag; dynamic five-band runtime stays disabled."""

    source_vertex_id: int
    incident_spine_edge_ids: tuple[int, int]
    source_face_ids: tuple[int, ...]
    policy: str = "STABLE_A10_IN_PLANE"


@dataclass(frozen=True)
class RailCornerPartition:
    partition_id: int
    source_vertex_id: int
    side_sign: int
    mode: str
    previous_channel_id: int
    next_channel_id: int
    previous_path_id: tuple
    next_path_id: tuple
    owner_face_id: int | None
    point: tuple[float, float, float]
    previous_lateral: tuple[float, float, float]
    next_lateral: tuple[float, float, float]
    miter_vector: tuple[float, float, float]
    miter_ratio: float
    alpha_limit: float
    s: float
    source_edge_ids: tuple[int, int]


@dataclass(frozen=True)
class RailCornerCell:
    cell_id: int
    partition_id: int
    owner_face_id: int
    spine_edge_id: int
    source_edge_ids: tuple[int, ...]
    boundary_path_ids: tuple[tuple, tuple]
    vertices: tuple[RailDomainVertex, ...]


@dataclass(frozen=True)
class RailFaceProvenance:
    component_id: int
    channel_id: int | None
    cell_id: int | None
    corner_partition_id: int | None
    source_face_id: int
    source_edge_ids: tuple[int, ...]
    boundary_path_ids: tuple[tuple, ...]
    route_ids: tuple[int, ...]
    station_keys: tuple[object, ...]


@dataclass
class RailDecalGeometryFace(DecalGeometryFace):
    """Materializer-compatible face with immutable rail provenance payload."""

    rail_provenance: RailFaceProvenance | None = None


@dataclass(frozen=True)
class PlanarRailGeometryPlan:
    rail_plan: object
    component: RailSpineComponent
    channels: tuple[RailChannel, ...]
    cap_traces: tuple[RailCapTrace, ...]
    corners: tuple[RailCornerUse, ...]
    corner_partitions: tuple[RailCornerPartition, ...]
    corner_cells: tuple[RailCornerCell, ...]
    boundary_paths: tuple[RailBoundaryPath, ...]
    path_reach_scales: tuple[tuple[tuple, float], ...]
    alpha_budget: float
    apex_limit: float
    split_angle: float

    @property
    def edge_indices(self):
        return self.component.edge_ids


def _add3(point, vector):
    return tuple(a + b for a, b in zip(point, vector))


def _sub3(point_a, point_b):
    return tuple(a - b for a, b in zip(point_a, point_b))


def _mul3(vector, scalar):
    return tuple(component * scalar for component in vector)


def _dot3(vector_a, vector_b):
    return sum(a * b for a, b in zip(vector_a, vector_b))


def _cross3(vector_a, vector_b):
    return (
        vector_a[1] * vector_b[2] - vector_a[2] * vector_b[1],
        vector_a[2] * vector_b[0] - vector_a[0] * vector_b[2],
        vector_a[0] * vector_b[1] - vector_a[1] * vector_b[0],
    )


def _length3(vector):
    return sqrt(_dot3(vector, vector))


def _normalized3(vector):
    length = _length3(vector)
    if length <= 0.0:
        return None
    return _mul3(vector, 1.0 / length)


def _position_by_vertex(rail_plan):
    return {vertex.vertex_id: vertex.position for vertex in rail_plan.vertices}


def _edge_by_id(rail_plan):
    return {edge.edge_id: edge for edge in rail_plan.edges}


def _face_by_id(rail_plan):
    return {face.face_id: face for face in rail_plan.faces}


def _station_position(route, station, vertices, edges):
    if station.kind == RailStationKind.VERTEX:
        return vertices[station.source_vertex_id]
    edge = edges[station.source_edge_id]
    point_a = vertices[edge.vertex_ids[0]]
    point_b = vertices[edge.vertex_ids[1]]
    parameter = float(station.edge_parameter)
    return _add3(point_a, _mul3(_sub3(point_b, point_a), parameter))


def _spine_components(rail_plan):
    edge_by_id = _edge_by_id(rail_plan)
    spine_edge_ids = {use.edge_id for use in rail_plan.spine_uses}
    by_vertex = defaultdict(set)
    for edge_id in spine_edge_ids:
        for vertex_id in edge_by_id[edge_id].vertex_ids:
            by_vertex[vertex_id].add(edge_id)
    remaining = set(spine_edge_ids)
    components = []
    while remaining:
        seed = min(remaining)
        pending = [seed]
        component = set()
        while pending:
            edge_id = pending.pop()
            if edge_id in component:
                continue
            component.add(edge_id)
            remaining.discard(edge_id)
            for vertex_id in edge_by_id[edge_id].vertex_ids:
                pending.extend(by_vertex[vertex_id].difference(component))
        components.append(frozenset(component))
    return tuple(sorted(components, key=lambda values: tuple(sorted(values))))


def _select_component(rail_plan, edge_indices):
    components = _spine_components(rail_plan)
    requested = frozenset(int(value) for value in edge_indices or ())
    if not requested:
        if len(components) != 1:
            raise _RailGeometryCompileError(
                "RAIL_GEOMETRY_COMPONENT_REQUIRED",
                edge_indices=tuple(
                    edge_id for component in components for edge_id in component
                ),
                details=(("component_count", len(components)),),
            )
        return components[0]
    known = frozenset(edge_id for component in components for edge_id in component)
    missing = requested.difference(known)
    if missing:
        raise _RailGeometryCompileError(
            "RAIL_GEOMETRY_EDGE_OUTSIDE_PLAN",
            edge_indices=missing,
        )
    matches = [component for component in components if component == requested]
    if len(matches) != 1:
        owner = next(
            (component for component in components if component.intersection(requested)),
            frozenset(),
        )
        raise _RailGeometryCompileError(
            "RAIL_GEOMETRY_COMPONENT_PARTIAL",
            edge_indices=requested,
            details=(("full_component", tuple(sorted(owner))),),
        )
    return matches[0]


def _compile_spine_component(rail_plan, component_edge_ids):
    edges = _edge_by_id(rail_plan)
    by_vertex = defaultdict(list)
    for edge_id in component_edge_ids:
        for vertex_id in edges[edge_id].vertex_ids:
            by_vertex[vertex_id].append(edge_id)
    branched = tuple(
        vertex_id for vertex_id, edge_ids in by_vertex.items() if len(edge_ids) > 2
    )
    if branched:
        raise _RailGeometryCompileError(
            "RAIL_GEOMETRY_BRANCH_UNSUPPORTED",
            edge_indices=component_edge_ids,
            vertex_indices=branched,
        )
    endpoints = sorted(
        vertex_id for vertex_id, edge_ids in by_vertex.items() if len(edge_ids) == 1
    )
    is_closed = not endpoints and all(
        len(edge_ids) == 2 for edge_ids in by_vertex.values()
    )
    uses = [
        (spine_use.edge_id, chain_use)
        for spine_use in rail_plan.spine_uses
        if spine_use.edge_id in component_edge_ids
        for chain_use in spine_use.chain_uses
    ]
    if len(endpoints) == 2:
        if not uses:
            raise _RailGeometryCompileError(
                "RAIL_GEOMETRY_OPEN_CHAIN_PROVENANCE_REQUIRED",
                edge_indices=component_edge_ids,
            )
        authority_edge, authority = min(
            uses,
            key=lambda item: (
                item[1].chain_ref,
                item[1].chain_edge_index,
                item[0],
            ),
        )
        backward_vertex = authority.oriented_vertex_ids[0]
        previous_edge = authority_edge
        while len(by_vertex[backward_vertex]) == 2:
            backward_edge = next(
                edge_id
                for edge_id in by_vertex[backward_vertex]
                if edge_id != previous_edge
            )
            edge = edges[backward_edge]
            backward_vertex = (
                edge.vertex_ids[1]
                if edge.vertex_ids[0] == backward_vertex
                else edge.vertex_ids[0]
            )
            previous_edge = backward_edge
        if backward_vertex not in endpoints:
            raise _RailGeometryCompileError(
                "RAIL_GEOMETRY_OPEN_CHAIN_PROVENANCE_INVALID",
                edge_indices=component_edge_ids,
                vertex_indices=authority.oriented_vertex_ids,
            )
        start_vertex = backward_vertex
        first_edge = None
    elif is_closed:
        if not uses:
            raise _RailGeometryCompileError(
                "RAIL_GEOMETRY_CLOSED_CHAIN_PROVENANCE_REQUIRED",
                edge_indices=component_edge_ids,
            )
        authority_edge, authority = min(
            uses,
            key=lambda item: (
                item[1].chain_ref,
                item[1].chain_edge_index,
                item[0],
            ),
        )
        start_vertex, oriented_next = authority.oriented_vertex_ids
        first_edge = authority_edge
        if oriented_next not in edges[first_edge].vertex_ids:
            first_edge = None
        if first_edge is None:
            raise _RailGeometryCompileError(
                "RAIL_GEOMETRY_CLOSED_CHAIN_PROVENANCE_INVALID",
                edge_indices=component_edge_ids,
                vertex_indices=authority.oriented_vertex_ids,
            )
    else:
        raise _RailGeometryCompileError(
            "RAIL_GEOMETRY_COMPONENT_ORDER_INVALID",
            edge_indices=component_edge_ids,
            vertex_indices=by_vertex,
        )

    vertex_ids = [start_vertex]
    ordered_edges = []
    previous_edge = None
    current_vertex = start_vertex
    while len(ordered_edges) < len(component_edge_ids):
        candidates = [
            edge_id
            for edge_id in sorted(by_vertex[current_vertex])
            if edge_id != previous_edge
        ]
        if not ordered_edges and first_edge is not None:
            candidates = [edge_id for edge_id in candidates if edge_id == first_edge]
        if len(candidates) != 1:
            raise _RailGeometryCompileError(
                "RAIL_GEOMETRY_COMPONENT_ORDER_INVALID",
                edge_indices=component_edge_ids,
                vertex_indices=(current_vertex,),
            )
        edge_id = candidates[0]
        edge = edges[edge_id]
        next_vertex = (
            edge.vertex_ids[1]
            if edge.vertex_ids[0] == current_vertex
            else edge.vertex_ids[0]
        )
        ordered_edges.append(edge_id)
        vertex_ids.append(next_vertex)
        previous_edge = edge_id
        current_vertex = next_vertex

    if is_closed:
        if vertex_ids[-1] != vertex_ids[0]:
            raise _RailGeometryCompileError(
                "RAIL_GEOMETRY_CLOSED_COMPONENT_INVALID",
                edge_indices=component_edge_ids,
                vertex_indices=vertex_ids,
            )
        # Начало и направление closed chain задаёт ChainRef provenance.  ID
        # вершины не является семантикой фазы и потому не может сдвигать s=0.
        vertex_ids = vertex_ids[:-1] + [vertex_ids[0]]

    stations = []
    intervals = []
    s = 0.0
    for station_index, vertex_id in enumerate(vertex_ids):
        stations.append(RailSpineStation(station_index, vertex_id, s))
        if station_index == len(ordered_edges):
            continue
        edge = edges[ordered_edges[station_index]]
        intervals.append(
            RailSpineInterval(
                interval_index=station_index,
                source_edge_id=edge.edge_id,
                from_station_index=station_index,
                to_station_index=station_index + 1,
                source_face_ids=edge.face_indices,
            )
        )
        s += edge.length
    return RailSpineComponent(
        component_id=min(component_edge_ids),
        edge_ids=tuple(ordered_edges),
        vertex_ids=tuple(vertex_ids),
        stations=tuple(stations),
        intervals=tuple(intervals),
        is_closed=is_closed,
    )


def _route_boundary_path(rail_plan, route):
    vertices = _position_by_vertex(rail_plan)
    edges = _edge_by_id(rail_plan)
    pieces = []
    for segment in route.segments:
        start_station = route.stations[segment.from_station_index]
        end_station = route.stations[segment.to_station_index]
        start_position = _station_position(route, start_station, vertices, edges)
        end_position = _station_position(route, end_station, vertices, edges)

        def boundary_vertex(station, position):
            if station.kind == RailStationKind.VERTEX:
                key = ("rail-source-vertex", station.source_vertex_id)
                feature = "SOURCE_VERTEX"
                feature_id = station.source_vertex_id
                source_edge_id = segment.edge_id
            else:
                key = (
                    "rail-source-edge-station",
                    station.source_edge_id,
                    station.edge_parameter,
                )
                feature = "SOURCE_EDGE"
                feature_id = (
                    station.source_edge_id,
                    station.edge_parameter,
                )
                source_edge_id = station.source_edge_id
            return RailBoundaryVertex(
                key=key,
                position=position,
                r=station.distance,
                source_feature=feature,
                source_feature_id=feature_id,
                source_edge_id=source_edge_id,
                route_id=route.route_id,
                station_index=station.station_index,
            )

        start = boundary_vertex(start_station, start_position)
        end = boundary_vertex(end_station, end_position)
        for owner_face_id in segment.source_face_ids:
            pieces.append(
                RailBoundaryPiece(
                    piece_index=segment.from_station_index,
                    owner_face_id=owner_face_id,
                    start=start,
                    end=end,
                    source_edge_ids=(segment.edge_id,),
                )
            )
    return RailBoundaryPath(
        path_id=("ROUTE", route.route_id),
        route_id=route.route_id,
        spine_vertex_id=route.key.side.spine_vertex_id,
        kind="ROUTE",
        pieces=tuple(sorted(pieces, key=lambda piece: (piece.piece_index, piece.owner_face_id))),
        termination=route.termination.value,
    )


def _face_edge_orientation(face, edge_id):
    try:
        edge_index = face.edge_ids.index(edge_id)
    except ValueError as exc:
        raise _RailGeometryCompileError(
            "RAIL_GEOMETRY_SPINE_FACE_INCIDENCE_INVALID",
            edge_indices=(edge_id,),
            face_indices=(face.face_id,),
        ) from exc
    return (
        face.vertex_ids[edge_index],
        face.vertex_ids[(edge_index + 1) % len(face.vertex_ids)],
    )


def _cap_direction(face, spine_edge, from_vertex_id, to_vertex_id, vertices):
    point_a = vertices[from_vertex_id]
    point_b = vertices[to_vertex_id]
    tangent = _normalized3(_sub3(point_b, point_a))
    if tangent is None:
        raise _RailGeometryCompileError(
            "RAIL_GEOMETRY_SPINE_EDGE_DEGENERATE",
            edge_indices=(spine_edge.edge_id,),
        )
    lateral = _cross3(face.normal, tangent)
    oriented_a, oriented_b = _face_edge_orientation(face, spine_edge.edge_id)
    if (oriented_a, oriented_b) == (from_vertex_id, to_vertex_id):
        direction = lateral
    elif (oriented_a, oriented_b) == (to_vertex_id, from_vertex_id):
        direction = _mul3(lateral, -1.0)
    else:
        raise _RailGeometryCompileError(
            "RAIL_GEOMETRY_SPINE_FACE_INCIDENCE_INVALID",
            edge_indices=(spine_edge.edge_id,),
            face_indices=(face.face_id,),
        )
    return _normalized3(direction)


def _line_edge_intersection(origin, direction, face_normal, point_a, point_b):
    """Пересечение ray и segment в owner plane без выбора кандидата."""

    transverse = _cross3(face_normal, direction)
    relative_a = _sub3(point_a, origin)
    relative_b = _sub3(point_b, origin)
    y_a = _dot3(relative_a, transverse)
    y_b = _dot3(relative_b, transverse)
    x_a = _dot3(relative_a, direction)
    x_b = _dot3(relative_b, direction)
    arithmetic = 1e-12 * max(1.0, abs(x_a), abs(x_b))
    if abs(y_a) <= arithmetic and abs(y_b) <= arithmetic:
        return "COLLINEAR", tuple(sorted((x_a, x_b)))
    denominator = y_a - y_b
    if abs(denominator) <= arithmetic:
        return None
    parameter = y_a / denominator
    if parameter < -arithmetic or parameter > 1.0 + arithmetic:
        return None
    parameter = max(0.0, min(1.0, parameter))
    x = x_a + (x_b - x_a) * parameter
    return "POINT", (x, parameter)


def _trace_virtual_cap(
    rail_plan,
    *,
    trace_id,
    route,
    spine_edge,
    from_vertex_id,
    to_vertex_id,
    initial_face_id,
):
    vertices = _position_by_vertex(rail_plan)
    edges = _edge_by_id(rail_plan)
    faces = _face_by_id(rail_plan)
    face = faces[initial_face_id]
    direction = _cap_direction(
        face,
        spine_edge,
        from_vertex_id,
        to_vertex_id,
        vertices,
    )
    if direction is None:
        raise _RailGeometryCompileError(
            "RAIL_GEOMETRY_CAP_DIRECTION_INVALID",
            edge_indices=(spine_edge.edge_id,),
            face_indices=(initial_face_id,),
        )
    origin = vertices[route.key.side.spine_vertex_id]
    current_face_id = initial_face_id
    current_r = 0.0
    current_vertex = RailBoundaryVertex(
        key=("rail-source-vertex", route.key.side.spine_vertex_id),
        position=origin,
        r=0.0,
        source_feature="SOURCE_VERTEX",
        source_feature_id=route.key.side.spine_vertex_id,
        source_edge_id=spine_edge.edge_id,
        route_id=route.route_id,
        station_index=0,
    )
    pieces = []
    termination = "ALPHA"
    visited_faces = set()
    rail_edge_ids = {
        segment.edge_id
        for candidate_route in rail_plan.routes
        for segment in candidate_route.segments
    }
    dam_vertex_ids = {
        station.source_vertex_id
        for event in rail_plan.events
        if event.kind.value == "DAM"
        for candidate_route in rail_plan.routes
        if candidate_route.route_id == event.route_id
        for station in candidate_route.stations
        if station.station_index == event.station_index
        and station.kind == RailStationKind.VERTEX
    }
    while current_r < rail_plan.alpha_budget:
        if current_face_id in visited_faces:
            termination = "DAM"
            break
        visited_faces.add(current_face_id)
        current_face = faces[current_face_id]
        if current_face.planarity_min_dot < _RAIL_PLANAR_DOT:
            raise _RailGeometryCompileError(
                "RAIL_GEOMETRY_SOURCE_FACE_NON_PLANAR",
                face_indices=(current_face_id,),
            )
        if _dot3(current_face.normal, face.normal) < _RAIL_PLANAR_DOT:
            termination = "FOLD"
            break

        candidate_groups = defaultdict(list)
        collinear = []
        for edge_id in current_face.edge_ids:
            edge = edges[edge_id]
            point_a = vertices[edge.vertex_ids[0]]
            point_b = vertices[edge.vertex_ids[1]]
            hit = _line_edge_intersection(
                origin,
                direction,
                current_face.normal,
                point_a,
                point_b,
            )
            if hit is None:
                continue
            kind, payload = hit
            if kind == "COLLINEAR":
                lower, upper = payload
                if upper > current_r:
                    collinear.append((upper, edge_id))
                continue
            distance, parameter = payload
            if distance > current_r:
                arithmetic = 1e-12 * max(1.0, abs(distance))
                if abs(parameter) <= arithmetic:
                    station_key = ("SOURCE_VERTEX", edge.vertex_ids[0])
                    distance = _dot3(
                        _sub3(vertices[edge.vertex_ids[0]], origin), direction
                    )
                elif abs(parameter - 1.0) <= arithmetic:
                    station_key = ("SOURCE_VERTEX", edge.vertex_ids[1])
                    distance = _dot3(
                        _sub3(vertices[edge.vertex_ids[1]], origin), direction
                    )
                else:
                    station_key = ("SOURCE_EDGE", edge_id, parameter)
                candidate_groups[station_key].append((distance, edge_id, parameter))
        exit_station_key = None
        if not candidate_groups and collinear:
            distance = min(value[0] for value in collinear)
            exit_edges = [value[1] for value in collinear if value[0] == distance]
            exit_parameter = None
        elif candidate_groups:
            stations = []
            for station_key, hits in candidate_groups.items():
                station_distance = hits[0][0]
                stations.append((station_distance, station_key, hits))
            stations.sort(key=lambda item: item[0])
            distance, station_key, hits = stations[0]
            exit_station_key = station_key
            arithmetic = 1e-12 * max(1.0, abs(distance))
            competing = [
                station
                for station in stations[1:]
                if abs(station[0] - distance) <= arithmetic
            ]
            if competing:
                termination = "DAM"
                break
            if station_key[0] == "SOURCE_VERTEX":
                non_collinear = []
                for _hit_distance, edge_id, _parameter in hits:
                    edge = edges[edge_id]
                    edge_direction = _normalized3(
                        _sub3(
                            vertices[edge.vertex_ids[1]],
                            vertices[edge.vertex_ids[0]],
                        )
                    )
                    if edge_direction is None:
                        continue
                    if abs(_dot3(edge_direction, direction)) < 1.0 - 1e-12:
                        non_collinear.append(edge_id)
                exit_edges = sorted(set(non_collinear))
                exit_parameter = None
            else:
                exit_edges = [hits[0][1]] if len(hits) == 1 else []
                exit_parameter = hits[0][2] if len(hits) == 1 else None
        else:
            termination = "DAM"
            break

        end_r = min(distance, rail_plan.alpha_budget)
        end_position = _add3(origin, _mul3(direction, end_r))
        exit_edge_id = exit_edges[0] if len(exit_edges) == 1 else None
        if end_r < distance:
            end_key = ("rail-cap-alpha", trace_id, len(pieces))
            source_feature = "CAP_TRACE"
            source_feature_id = (trace_id, len(pieces), end_r)
            source_edge_id = spine_edge.edge_id
        elif exit_station_key is not None and exit_station_key[0] == "SOURCE_VERTEX":
            source_vertex_id = int(exit_station_key[1])
            end_position = vertices[source_vertex_id]
            end_key = ("rail-source-vertex", source_vertex_id)
            source_feature = "SOURCE_VERTEX"
            source_feature_id = source_vertex_id
            source_edge_id = exit_edge_id
        elif exit_edge_id is not None:
            edge = edges[exit_edge_id]
            point_a = vertices[edge.vertex_ids[0]]
            point_b = vertices[edge.vertex_ids[1]]
            edge_vector = _sub3(point_b, point_a)
            denominator = _dot3(edge_vector, edge_vector)
            parameter = (
                _dot3(_sub3(end_position, point_a), edge_vector) / denominator
                if denominator > 0.0
                else 0.0
            )
            if parameter == 0.0:
                end_key = ("rail-source-vertex", edge.vertex_ids[0])
                source_feature = "SOURCE_VERTEX"
                source_feature_id = edge.vertex_ids[0]
            elif parameter == 1.0:
                end_key = ("rail-source-vertex", edge.vertex_ids[1])
                source_feature = "SOURCE_VERTEX"
                source_feature_id = edge.vertex_ids[1]
            else:
                end_key = ("rail-source-edge-station", exit_edge_id, parameter)
                source_feature = "SOURCE_EDGE"
                source_feature_id = (exit_edge_id, parameter)
            source_edge_id = exit_edge_id
        elif end_r == rail_plan.alpha_budget:
            end_key = ("rail-cap-alpha", trace_id, len(pieces))
            source_feature = "CAP_TRACE"
            source_feature_id = (trace_id, len(pieces), end_r)
            source_edge_id = spine_edge.edge_id
        else:
            end_key = ("rail-cap-dam", trace_id, len(pieces))
            source_feature = "CAP_TRACE"
            source_feature_id = (trace_id, len(pieces), end_r)
            source_edge_id = spine_edge.edge_id
        end_vertex = RailBoundaryVertex(
            key=end_key,
            position=end_position,
            r=end_r,
            source_feature=source_feature,
            source_feature_id=source_feature_id,
            source_edge_id=source_edge_id,
            route_id=route.route_id,
            station_index=None,
        )
        pieces.append(
            RailBoundaryPiece(
                piece_index=len(pieces),
                owner_face_id=current_face_id,
                start=current_vertex,
                end=end_vertex,
                source_edge_ids=tuple(
                    sorted({spine_edge.edge_id, *(exit_edges or ())})
                ),
            )
        )
        current_vertex = end_vertex
        current_r = end_r
        if current_r >= rail_plan.alpha_budget:
            termination = "ALPHA"
            break
        if len(exit_edges) != 1:
            termination = "DAM"
            break
        exit_edge = edges[exit_edges[0]]
        hit_dam_vertex = (
            exit_station_key is not None
            and exit_station_key[0] == "SOURCE_VERTEX"
            and int(exit_station_key[1]) in dam_vertex_ids
        )
        if exit_edge.is_pchain or exit_edge.is_spine:
            termination = "PCHAIN"
            break
        if exit_edge.edge_id in rail_edge_ids or hit_dam_vertex:
            termination = "DAM"
            break
        next_faces = tuple(
            face_id
            for face_id in exit_edge.face_indices
            if face_id != current_face_id
        )
        if len(next_faces) != 1:
            termination = "MESH_BORDER" if not next_faces else "DAM"
            break
        next_face = faces[next_faces[0]]
        if _dot3(next_face.normal, face.normal) < _RAIL_PLANAR_DOT:
            termination = "FOLD"
            break
        current_face_id = next_faces[0]

    trace = RailCapTrace(
        trace_id=trace_id,
        route_id=route.route_id,
        spine_vertex_id=route.key.side.spine_vertex_id,
        spine_edge_id=spine_edge.edge_id,
        initial_face_id=initial_face_id,
        direction=direction,
        pieces=tuple(pieces),
        termination=termination,
    )
    return trace, RailBoundaryPath(
        path_id=("CAP", trace_id),
        route_id=route.route_id,
        spine_vertex_id=route.key.side.spine_vertex_id,
        kind="CAP",
        pieces=trace.pieces,
        termination=trace.termination,
        cap_trace_id=trace.trace_id,
    )


def _route_for_face(rail_plan, spine_vertex_id, face_id):
    candidates = [
        route
        for route in rail_plan.routes
        if route.key.side.spine_vertex_id == spine_vertex_id
        and face_id in route.key.side.source_face_ids
    ]
    if len(candidates) != 1:
        raise _RailGeometryCompileError(
            "RAIL_GEOMETRY_BOUNDARY_AMBIGUOUS",
            vertex_indices=(spine_vertex_id,),
            face_indices=(face_id,),
            route_ids=(route.route_id for route in candidates),
            details=(("candidate_count", len(candidates)),),
        )
    route = candidates[0]
    if route.termination == RailTermination.MERGE:
        raise _RailGeometryCompileError(
            "RAIL_GEOMETRY_MERGE_UNSUPPORTED",
            vertex_indices=(spine_vertex_id,),
            face_indices=(face_id,),
            route_ids=(route.route_id,),
        )
    if route.termination == RailTermination.POLE:
        raise _RailGeometryCompileError(
            "RAIL_GEOMETRY_POLE_UNSUPPORTED",
            vertex_indices=(spine_vertex_id,),
            face_indices=(face_id,),
            route_ids=(route.route_id,),
        )
    return route


def _path_for_endpoint(
    rail_plan,
    component,
    interval,
    face_id,
    endpoint_station_index,
    trace_cache,
    cap_traces,
):
    station = component.stations[endpoint_station_index]
    route = _route_for_face(rail_plan, station.source_vertex_id, face_id)
    if route.key.side.start_edge_id >= 0:
        faces = _face_by_id(rail_plan)
        owner_normal = faces[face_id].normal
        route_face_ids = {
            source_face_id
            for segment in route.segments
            for source_face_id in segment.source_face_ids
        }
        curved_faces = tuple(
            sorted(
                source_face_id
                for source_face_id in route_face_ids
                if _dot3(faces[source_face_id].normal, owner_normal)
                < _RAIL_PLANAR_DOT
            )
        )
        if curved_faces:
            raise _RailGeometryCompileError(
                "RAIL_GEOMETRY_CURVED_OWNER_UNSUPPORTED",
                edge_indices=(interval.source_edge_id,),
                vertex_indices=(station.source_vertex_id,),
                face_indices=(face_id, *curved_faces),
                route_ids=(route.route_id,),
            )
        return _route_boundary_path(rail_plan, route)
    if not (
        route.termination == RailTermination.DAM
        and len(route.stations) == 1
        and not route.segments
    ):
        raise _RailGeometryCompileError(
            "RAIL_GEOMETRY_VIRTUAL_CAP_SOURCE_INVALID",
            edge_indices=(interval.source_edge_id,),
            vertex_indices=(station.source_vertex_id,),
            face_indices=(face_id,),
            route_ids=(route.route_id,),
        )

    cache_key = (route.route_id, interval.source_edge_id, face_id)
    cached = trace_cache.get(cache_key)
    if cached is not None:
        return cached
    edge = _edge_by_id(rail_plan)[interval.source_edge_id]
    from_vertex_id = component.vertex_ids[interval.from_station_index]
    to_vertex_id = component.vertex_ids[interval.to_station_index]
    trace, path = _trace_virtual_cap(
        rail_plan,
        trace_id=len(cap_traces),
        route=route,
        spine_edge=edge,
        from_vertex_id=from_vertex_id,
        to_vertex_id=to_vertex_id,
        initial_face_id=face_id,
    )
    if not trace.pieces:
        raise _RailGeometryCompileError(
            "RAIL_GEOMETRY_VIRTUAL_CAP_EMPTY",
            edge_indices=(interval.source_edge_id,),
            vertex_indices=(station.source_vertex_id,),
            face_indices=(face_id,),
            route_ids=(route.route_id,),
        )
    cap_traces.append(trace)
    trace_cache[cache_key] = path
    return path


def _piece_for_face(path, face_id, minimum_index):
    candidates = [
        piece
        for piece in path.pieces
        if piece.owner_face_id == face_id and piece.piece_index >= minimum_index
    ]
    if not candidates:
        return None
    first_index = min(piece.piece_index for piece in candidates)
    exact = [piece for piece in candidates if piece.piece_index == first_index]
    if len(exact) != 1:
        raise _RailGeometryCompileError(
            "RAIL_GEOMETRY_BOUNDARY_PIECE_AMBIGUOUS",
            face_indices=(face_id,),
            route_ids=(path.route_id,),
        )
    return exact[0]


def _next_face(path_a, piece_a, path_b, piece_b, current_face_id):
    next_a = {
        piece.owner_face_id
        for piece in path_a.pieces
        if piece.piece_index == piece_a.piece_index + 1
    }
    next_b = {
        piece.owner_face_id
        for piece in path_b.pieces
        if piece.piece_index == piece_b.piece_index + 1
    }
    candidates = sorted(next_a.intersection(next_b).difference({current_face_id}))
    remaining_a = next_a.difference({current_face_id})
    remaining_b = next_b.difference({current_face_id})
    future_a = any(
        piece.piece_index > piece_a.piece_index for piece in path_a.pieces
    )
    future_b = any(
        piece.piece_index > piece_b.piece_index for piece in path_b.pieces
    )
    if not candidates and future_a and future_b:
        raise _RailGeometryCompileError(
            "RAIL_GEOMETRY_CHANNEL_COVERAGE_DESYNC",
            face_indices=tuple(sorted(remaining_a.union(remaining_b))),
            route_ids=(path_a.route_id, path_b.route_id),
            details=(
                ("path_a_faces", tuple(sorted(remaining_a))),
                ("path_b_faces", tuple(sorted(remaining_b))),
            ),
        )
    if len(candidates) > 1:
        raise _RailGeometryCompileError(
            "RAIL_GEOMETRY_CHANNEL_NEXT_FACE_AMBIGUOUS",
            face_indices=candidates,
            route_ids=(path_a.route_id, path_b.route_id),
        )
    return candidates[0] if candidates else None


def _domain_vertex(boundary_vertex, *, s, face_id, path_id):
    return RailDomainVertex(
        key=boundary_vertex.key,
        position=boundary_vertex.position,
        s=float(s),
        r=float(boundary_vertex.r),
        boundary_path_id=path_id,
        source_feature=boundary_vertex.source_feature,
        source_feature_id=boundary_vertex.source_feature_id,
        source_edge_id=boundary_vertex.source_edge_id,
        source_face_id=face_id,
        route_ids=(boundary_vertex.route_id,),
        station_index=boundary_vertex.station_index,
    )


def _polygon_normal(vertices):
    if len(vertices) < 3:
        return None
    origin = vertices[0].position
    total = (0.0, 0.0, 0.0)
    for index in range(1, len(vertices) - 1):
        total = _add3(
            total,
            _cross3(
                _sub3(vertices[index].position, origin),
                _sub3(vertices[index + 1].position, origin),
            ),
        )
    return _normalized3(total)


def _compile_channel_cells(
    *,
    rail_plan,
    channel_id,
    component,
    interval,
    initial_face_id,
    path_a,
    path_b,
    first_cell_id,
):
    faces = _face_by_id(rail_plan)
    current_face_id = initial_face_id
    minimum_a = 0
    minimum_b = 0
    visited = set()
    cells = []
    s_a = component.stations[interval.from_station_index].s
    s_b = component.stations[interval.to_station_index].s
    while current_face_id is not None:
        if current_face_id in visited:
            raise _RailGeometryCompileError(
                "RAIL_GEOMETRY_CHANNEL_FACE_CYCLE",
                face_indices=(current_face_id,),
                route_ids=(path_a.route_id, path_b.route_id),
            )
        visited.add(current_face_id)
        face = faces[current_face_id]
        if face.planarity_min_dot < _RAIL_PLANAR_DOT:
            raise _RailGeometryCompileError(
                "RAIL_GEOMETRY_SOURCE_FACE_NON_PLANAR",
                face_indices=(current_face_id,),
            )
        initial_face = faces[initial_face_id]
        if _dot3(face.normal, initial_face.normal) < _RAIL_PLANAR_DOT:
            raise _RailGeometryCompileError(
                "RAIL_GEOMETRY_CURVED_OWNER_UNSUPPORTED",
                face_indices=(initial_face_id, current_face_id),
                route_ids=(path_a.route_id, path_b.route_id),
            )
        piece_a = _piece_for_face(path_a, current_face_id, minimum_a)
        piece_b = _piece_for_face(path_b, current_face_id, minimum_b)
        if piece_a is None or piece_b is None:
            if not cells:
                raise _RailGeometryCompileError(
                    "RAIL_GEOMETRY_CHANNEL_BOUNDARY_MISSING",
                    edge_indices=(interval.source_edge_id,),
                    face_indices=(current_face_id,),
                    route_ids=(path_a.route_id, path_b.route_id),
                )
            break
        minimum_a = piece_a.piece_index + 1
        minimum_b = piece_b.piece_index + 1
        vertices = (
            _domain_vertex(
                piece_a.start,
                s=s_a,
                face_id=current_face_id,
                path_id=path_a.path_id,
            ),
            _domain_vertex(
                piece_b.start,
                s=s_b,
                face_id=current_face_id,
                path_id=path_b.path_id,
            ),
            _domain_vertex(
                piece_b.end,
                s=s_b,
                face_id=current_face_id,
                path_id=path_b.path_id,
            ),
            _domain_vertex(
                piece_a.end,
                s=s_a,
                face_id=current_face_id,
                path_id=path_a.path_id,
            ),
        )
        polygon_normal = _polygon_normal(vertices)
        if polygon_normal is None:
            raise _RailGeometryCompileError(
                "RAIL_GEOMETRY_CHANNEL_CELL_DEGENERATE",
                edge_indices=(interval.source_edge_id,),
                face_indices=(current_face_id,),
                route_ids=(path_a.route_id, path_b.route_id),
            )
        if _dot3(polygon_normal, face.normal) < 0.0:
            vertices = tuple(reversed(vertices))
        cells.append(
            RailChannelCell(
                cell_id=first_cell_id + len(cells),
                channel_id=channel_id,
                owner_face_id=current_face_id,
                spine_edge_id=interval.source_edge_id,
                source_edge_ids=tuple(
                    sorted(
                        {
                            interval.source_edge_id,
                            *piece_a.source_edge_ids,
                            *piece_b.source_edge_ids,
                        }
                    )
                ),
                boundary_path_ids=(path_a.path_id, path_b.path_id),
                vertices=vertices,
            )
        )
        current_face_id = _next_face(
            path_a,
            piece_a,
            path_b,
            piece_b,
            current_face_id,
        )
    return tuple(cells)


def _side_sign(face, spine_edge_id, from_vertex_id, to_vertex_id):
    oriented = _face_edge_orientation(face, spine_edge_id)
    return 1 if oriented == (to_vertex_id, from_vertex_id) else -1


def _unit_miter_vector(tangent_prev, tangent_next, lateral_prev, lateral_next, normal):
    denominator = _dot3(_cross3(tangent_prev, tangent_next), normal)
    if denominator == 0.0:
        return None
    parameter = _dot3(
        _cross3(_sub3(lateral_next, lateral_prev), tangent_next),
        normal,
    ) / denominator
    return _add3(lateral_prev, _mul3(tangent_prev, parameter))


def _corner_extrusion_angle(tangent_prev, tangent_next):
    value = _dot3(_mul3(tangent_prev, -1.0), tangent_next)
    return acos(max(-1.0, min(1.0, value)))


def _path_initial_direction(path, corner_vertex_id):
    anchor_key = ("rail-source-vertex", corner_vertex_id)
    candidates = [
        piece
        for piece in path.pieces
        if piece.start.key == anchor_key
        and piece.end.key != anchor_key
    ]
    if not candidates:
        return None
    first_index = min(piece.piece_index for piece in candidates)
    first = [
        piece for piece in candidates if piece.piece_index == first_index
    ]
    directions = {
        _normalized3(_sub3(piece.end.position, piece.start.position))
        for piece in first
    }
    directions.discard(None)
    return next(iter(directions)) if len(directions) == 1 else None


def _compile_corner_partitions(
    rail_plan,
    component,
    channels,
    path_by_id,
    apex_limit,
    split_angle,
):
    edges = _edge_by_id(rail_plan)
    faces = _face_by_id(rail_plan)
    vertices = _position_by_vertex(rail_plan)
    channels_by_edge_sign = defaultdict(list)
    for channel in channels:
        channels_by_edge_sign[(channel.spine_edge_id, channel.side_sign)].append(
            channel
        )
    station_by_vertex = {
        station.source_vertex_id: station for station in component.stations
    }
    corner_indices = (
        range(len(component.intervals))
        if component.is_closed
        else range(1, len(component.intervals))
    )
    partitions = []
    for next_interval_index in corner_indices:
        previous_interval_index = (
            next_interval_index - 1
            if next_interval_index > 0
            else len(component.intervals) - 1
        )
        previous_interval = component.intervals[previous_interval_index]
        next_interval = component.intervals[next_interval_index]
        corner_vertex_id = component.vertex_ids[next_interval_index]
        point = vertices[corner_vertex_id]
        previous_start = vertices[
            component.vertex_ids[previous_interval.from_station_index]
        ]
        next_end = vertices[
            component.vertex_ids[next_interval.to_station_index]
        ]
        tangent_prev = _normalized3(_sub3(point, previous_start))
        tangent_next = _normalized3(_sub3(next_end, point))
        if tangent_prev is None or tangent_next is None:
            raise _RailGeometryCompileError(
                "RAIL_GEOMETRY_CORNER_TANGENT_INVALID",
                edge_indices=(
                    previous_interval.source_edge_id,
                    next_interval.source_edge_id,
                ),
                vertex_indices=(corner_vertex_id,),
            )
        for side_sign in (-1, 1):
            previous_candidates = channels_by_edge_sign.get(
                (previous_interval.source_edge_id, side_sign), ()
            )
            next_candidates = channels_by_edge_sign.get(
                (next_interval.source_edge_id, side_sign), ()
            )
            if len(previous_candidates) != 1 or len(next_candidates) != 1:
                continue
            previous_channel = previous_candidates[0]
            next_channel = next_candidates[0]
            previous_path = path_by_id[previous_channel.to_path_id]
            next_path = path_by_id[next_channel.from_path_id]
            previous_face = faces[previous_channel.initial_face_id]
            next_face = faces[next_channel.initial_face_id]
            if _dot3(previous_face.normal, next_face.normal) < _RAIL_PLANAR_DOT:
                raise _RailGeometryCompileError(
                    "RAIL_GEOMETRY_CURVED_OWNER_UNSUPPORTED",
                    face_indices=(previous_face.face_id, next_face.face_id),
                    vertex_indices=(corner_vertex_id,),
                )

            extrusion_angle = _corner_extrusion_angle(
                tangent_prev,
                tangent_next,
            )
            if extrusion_angle < split_angle:
                raise _RailGeometryCompileError(
                    "RAIL_GEOMETRY_A10_POLICY_UNSUPPORTED",
                    edge_indices=(
                        previous_interval.source_edge_id,
                        next_interval.source_edge_id,
                    ),
                    vertex_indices=(corner_vertex_id,),
                    details=(
                        ("extrusion_angle", extrusion_angle),
                        ("split_angle", split_angle),
                    ),
                )

            cap_pair = (
                previous_path.kind == "CAP"
                and next_path.kind == "CAP"
            )
            shared_route = (
                previous_path.kind == "ROUTE"
                and next_path.kind == "ROUTE"
                and previous_path.path_id == next_path.path_id
            )
            if cap_pair:
                lateral_prev = _path_initial_direction(
                    previous_path,
                    corner_vertex_id,
                )
                lateral_next = _path_initial_direction(
                    next_path,
                    corner_vertex_id,
                )
                direction_failure = "RAIL_GEOMETRY_CORNER_CAP_DIRECTION_INVALID"
            elif shared_route:
                # Offset-lines обеих incident strips сходятся на одной
                # compile-static route; route лишь несёт их общий miter.
                lateral_prev = _mul3(
                    _cross3(previous_face.normal, tangent_prev),
                    -side_sign,
                )
                lateral_next = _mul3(
                    _cross3(next_face.normal, tangent_next),
                    -side_sign,
                )
                direction_failure = "RAIL_GEOMETRY_CORNER_ROUTE_DIRECTION_INVALID"
            else:
                denominator = _dot3(
                    _cross3(tangent_prev, tangent_next),
                    previous_face.normal,
                )
                if denominator == 0.0 and _dot3(tangent_prev, tangent_next) > 0.0:
                    continue
                raise _RailGeometryCompileError(
                    "RAIL_GEOMETRY_CORNER_PATH_UNSUPPORTED",
                    edge_indices=(
                        previous_interval.source_edge_id,
                        next_interval.source_edge_id,
                    ),
                    vertex_indices=(corner_vertex_id,),
                    route_ids=(previous_path.route_id, next_path.route_id),
                    details=(
                        ("previous_kind", previous_path.kind),
                        ("next_kind", next_path.kind),
                    ),
                )
            lateral_prev = _normalized3(lateral_prev)
            lateral_next = _normalized3(lateral_next)
            if lateral_prev is None or lateral_next is None:
                raise _RailGeometryCompileError(
                    direction_failure,
                    vertex_indices=(corner_vertex_id,),
                )
            miter_vector = _unit_miter_vector(
                tangent_prev,
                tangent_next,
                lateral_prev,
                lateral_next,
                previous_face.normal,
            )
            if miter_vector is None:
                continue
            miter_ratio = _length3(miter_vector)
            if miter_ratio > apex_limit:
                raise _RailGeometryCompileError(
                    "RAIL_GEOMETRY_A10_CORNER_UNSUPPORTED",
                    edge_indices=(
                        previous_interval.source_edge_id,
                        next_interval.source_edge_id,
                    ),
                    vertex_indices=(corner_vertex_id,),
                    details=(("miter_ratio", miter_ratio),),
                )
            if shared_route:
                route_direction = _path_initial_direction(
                    previous_path,
                    corner_vertex_id,
                )
                unit_miter = _normalized3(miter_vector)
                if (
                    route_direction is None
                    or unit_miter is None
                    or _dot3(route_direction, unit_miter) < _RAIL_PLANAR_DOT
                ):
                    raise _RailGeometryCompileError(
                        "RAIL_GEOMETRY_CORNER_ROUTE_MITER_MISMATCH",
                        edge_indices=(
                            previous_interval.source_edge_id,
                            next_interval.source_edge_id,
                        ),
                        vertex_indices=(corner_vertex_id,),
                        route_ids=(previous_path.route_id,),
                    )
                mode = "SHARED_MITER_ROUTE"
                owner_face_id = None
            elif previous_channel.initial_face_id == next_channel.initial_face_id:
                mode = "INNER_DIVIDER"
                owner_face_id = previous_channel.initial_face_id
            else:
                mode = "OUTER_FILL"
                incident_selected_faces = set(
                    edges[previous_interval.source_edge_id].face_indices
                ).union(edges[next_interval.source_edge_id].face_indices)
                owner_candidates = [
                    face.face_id
                    for face in rail_plan.faces
                    if corner_vertex_id in face.vertex_ids
                    and face.face_id not in incident_selected_faces
                    and _dot3(face.normal, previous_face.normal)
                    >= _RAIL_PLANAR_DOT
                ]
                if len(owner_candidates) > 1:
                    raise _RailGeometryCompileError(
                        "RAIL_GEOMETRY_CORNER_OWNER_AMBIGUOUS",
                        vertex_indices=(corner_vertex_id,),
                        face_indices=owner_candidates,
                    )
                owner_face_id = owner_candidates[0] if owner_candidates else None
            partitions.append(
                RailCornerPartition(
                    partition_id=len(partitions),
                    source_vertex_id=corner_vertex_id,
                    side_sign=side_sign,
                    mode=mode,
                    previous_channel_id=previous_channel.channel_id,
                    next_channel_id=next_channel.channel_id,
                    previous_path_id=previous_path.path_id,
                    next_path_id=next_path.path_id,
                    owner_face_id=owner_face_id,
                    point=point,
                    previous_lateral=lateral_prev,
                    next_lateral=lateral_next,
                    miter_vector=miter_vector,
                    miter_ratio=miter_ratio,
                    alpha_limit=0.0,
                    s=station_by_vertex[corner_vertex_id].s,
                    source_edge_ids=(
                        previous_interval.source_edge_id,
                        next_interval.source_edge_id,
                    ),
                )
            )
    return tuple(partitions)


def _path_reach(path):
    return max(
        (vertex.r for piece in path.pieces for vertex in (piece.start, piece.end)),
        default=0.0,
    )


def _compile_path_reach_scales(path_by_id, partitions):
    scales = {path_id: 1.0 for path_id in path_by_id}
    for partition in partitions:
        if partition.mode != "SHARED_MITER_ROUTE":
            continue
        path_id = partition.previous_path_id
        existing = scales[path_id]
        if existing != 1.0 and existing != partition.miter_ratio:
            raise _RailGeometryCompileError(
                "RAIL_GEOMETRY_CORNER_ROUTE_SCALE_CONFLICT",
                vertex_indices=(partition.source_vertex_id,),
                route_ids=(path_by_id[path_id].route_id,),
                details=(
                    ("existing_scale", existing),
                    ("new_scale", partition.miter_ratio),
                ),
            )
        scales[path_id] = partition.miter_ratio
    return scales


def _normalize_channels(channels, path_by_id, path_scales):
    normalized = []
    for channel in channels:
        path_ids = (channel.from_path_id, channel.to_path_id)
        alpha_limit = min(
            _path_reach(path_by_id[path_id]) / path_scales[path_id]
            for path_id in path_ids
        )
        cells = tuple(
            replace(
                cell,
                vertices=tuple(
                    replace(
                        vertex,
                        r=(
                            vertex.r
                            / path_scales[vertex.boundary_path_id]
                        ),
                    )
                    for vertex in cell.vertices
                ),
            )
            for cell in channel.cells
        )
        normalized.append(
            replace(channel, cells=cells, alpha_limit=alpha_limit)
        )
    return tuple(normalized)


def _synchronize_corner_limits(channels, partitions, path_by_id, path_scales):
    by_channel_id = {channel.channel_id: channel for channel in channels}
    changed = True
    while changed:
        changed = False
        for partition in partitions:
            previous = by_channel_id[partition.previous_channel_id]
            next_channel = by_channel_id[partition.next_channel_id]
            shared_limit = min(previous.alpha_limit, next_channel.alpha_limit)
            if previous.alpha_limit != shared_limit:
                by_channel_id[previous.channel_id] = replace(
                    previous,
                    alpha_limit=shared_limit,
                )
                changed = True
            if next_channel.alpha_limit != shared_limit:
                by_channel_id[next_channel.channel_id] = replace(
                    next_channel,
                    alpha_limit=shared_limit,
                )
                changed = True
    synchronized_partitions = tuple(
        replace(
            partition,
            alpha_limit=min(
                by_channel_id[partition.previous_channel_id].alpha_limit,
                by_channel_id[partition.next_channel_id].alpha_limit,
                *(
                    _path_reach(path_by_id[path_id]) / path_scales[path_id]
                    for path_id in (
                        partition.previous_path_id,
                        partition.next_path_id,
                    )
                ),
            ),
        )
        for partition in partitions
    )
    return (
        tuple(by_channel_id[index] for index in sorted(by_channel_id)),
        synchronized_partitions,
    )


def _effective_alpha_budget(rail_plan, path_by_id, path_scales):
    limits = []
    requested = float(rail_plan.alpha_budget)
    arithmetic = 1e-12 * max(1.0, requested)
    for path_id, path in path_by_id.items():
        reach = _path_reach(path)
        budget_limited = (
            path.termination == "ALPHA"
            or reach >= requested - arithmetic
        )
        if budget_limited:
            limits.append(reach / path_scales[path_id])
    effective = min((requested, *limits))
    if not effective > 0.0:
        raise _RailGeometryCompileError(
            "RAIL_GEOMETRY_COMPILE_REACH_EMPTY",
            route_ids=(path.route_id for path in path_by_id.values()),
        )
    return effective


def _corner_coefficients(partition, point, normal):
    relative = _sub3(point, partition.point)
    denominator = _dot3(
        _cross3(partition.previous_lateral, partition.next_lateral),
        normal,
    )
    if denominator == 0.0:
        raise _RailGeometryCompileError(
            "RAIL_GEOMETRY_CORNER_DOMAIN_DEGENERATE",
            vertex_indices=(partition.source_vertex_id,),
        )
    previous_coefficient = _dot3(
        _cross3(relative, partition.next_lateral),
        normal,
    ) / denominator
    next_coefficient = _dot3(
        _cross3(partition.previous_lateral, relative),
        normal,
    ) / denominator
    return previous_coefficient, next_coefficient


def _corner_radius(partition, point, normal):
    previous, next_value = _corner_coefficients(partition, point, normal)
    miter_previous, miter_next = _corner_coefficients(
        partition,
        _add3(partition.point, partition.miter_vector),
        normal,
    )
    if miter_previous <= 0.0 or miter_next <= 0.0:
        raise _RailGeometryCompileError(
            "RAIL_GEOMETRY_CORNER_DOMAIN_DEGENERATE",
            vertex_indices=(partition.source_vertex_id,),
            details=(
                ("miter_previous", miter_previous),
                ("miter_next", miter_next),
            ),
        )
    previous_cap = previous + (
        (1.0 - miter_previous) / miter_next
    ) * next_value
    next_cap = (
        (1.0 - miter_next) / miter_previous
    ) * previous + next_value
    return max(previous_cap, next_cap)


def _corner_path_station_lookup(path_by_id, partition):
    lookup = defaultdict(list)
    for path_id in (
        partition.previous_path_id,
        partition.next_path_id,
    ):
        for boundary_vertex in _ordered_path_vertices(path_by_id[path_id]):
            lookup[boundary_vertex.position].append((path_id, boundary_vertex))
    return lookup


def _source_face_is_convex(face, positions):
    """Convexity source-cycle без числового выбора поведения."""

    signs = set()
    points = tuple(positions[vertex_id] for vertex_id in face.vertex_ids)
    if len(points) < 3:
        return False
    for index, point in enumerate(points):
        next_point = points[(index + 1) % len(points)]
        following_point = points[(index + 2) % len(points)]
        turn = _dot3(
            _cross3(
                _sub3(next_point, point),
                _sub3(following_point, next_point),
            ),
            face.normal,
        )
        if turn > 0.0:
            signs.add(1)
        elif turn < 0.0:
            signs.add(-1)
        if len(signs) > 1:
            return False
    return bool(signs)


def _project_polygon(points, normal):
    """Детерминированная 2D-проекция одной compile-плоскости."""

    dropped_axis = max(
        range(3),
        key=lambda axis: (abs(normal[axis]), -axis),
    )
    axes = tuple(axis for axis in range(3) if axis != dropped_axis)
    return tuple((point[axes[0]], point[axes[1]]) for point in points)


def _orientation2(point_a, point_b, point_c):
    return (
        (point_b[0] - point_a[0]) * (point_c[1] - point_a[1])
        - (point_b[1] - point_a[1]) * (point_c[0] - point_a[0])
    )


def _polygon_arithmetic_tolerance(*polygons):
    coordinates = tuple(
        coordinate
        for polygon in polygons
        for point in polygon
        for coordinate in point
    )
    scale = max((1.0, *(abs(value) for value in coordinates)))
    # Только запас arithmetic representation. Он расширяет overlap и поэтому
    # не может пропустить опасную face; максимум даст безопасный fallback.
    return 1e-12 * scale * scale


def _point_on_segment2(point, point_a, point_b, tolerance):
    if abs(_orientation2(point_a, point_b, point)) > tolerance:
        return False
    coordinate_scale = max(
        1.0,
        *(abs(value) for item in (point, point_a, point_b) for value in item),
    )
    coordinate_tolerance = 1e-12 * coordinate_scale
    return (
        min(point_a[0], point_b[0]) - coordinate_tolerance
        <= point[0]
        <= max(point_a[0], point_b[0]) + coordinate_tolerance
        and min(point_a[1], point_b[1]) - coordinate_tolerance
        <= point[1]
        <= max(point_a[1], point_b[1]) + coordinate_tolerance
    )


def _point_in_polygon2(point, polygon, tolerance):
    """General simple-polygon containment, boundary included."""

    for point_a, point_b in zip(polygon, polygon[1:] + polygon[:1]):
        if _point_on_segment2(point, point_a, point_b, tolerance):
            return True
    inside = False
    for point_a, point_b in zip(polygon, polygon[1:] + polygon[:1]):
        if (point_a[1] > point[1]) == (point_b[1] > point[1]):
            continue
        crossing_x = point_a[0] + (
            (point[1] - point_a[1])
            * (point_b[0] - point_a[0])
            / (point_b[1] - point_a[1])
        )
        if crossing_x >= point[0]:
            inside = not inside
    return inside


def _segments_intersect2(point_a, point_b, point_c, point_d, tolerance):
    orientations = (
        _orientation2(point_a, point_b, point_c),
        _orientation2(point_a, point_b, point_d),
        _orientation2(point_c, point_d, point_a),
        _orientation2(point_c, point_d, point_b),
    )

    def sign(value):
        if value > tolerance:
            return 1
        if value < -tolerance:
            return -1
        return 0

    signs = tuple(sign(value) for value in orientations)
    if signs[0] * signs[1] < 0 and signs[2] * signs[3] < 0:
        return True
    return any(
        orientation_sign == 0
        and _point_on_segment2(point, segment_a, segment_b, tolerance)
        for orientation_sign, point, segment_a, segment_b in (
            (signs[0], point_c, point_a, point_b),
            (signs[1], point_d, point_a, point_b),
            (signs[2], point_a, point_c, point_d),
            (signs[3], point_b, point_c, point_d),
        )
    )


def _coplanar_polygons_overlap(points_a, points_b, normal):
    """Conservative overlap без Sutherland-Hodgman над concave cycle."""

    polygon_a = _project_polygon(points_a, normal)
    polygon_b = _project_polygon(points_b, normal)
    tolerance = _polygon_arithmetic_tolerance(polygon_a, polygon_b)
    if any(
        _point_in_polygon2(point, polygon_b, tolerance)
        for point in polygon_a
    ):
        return True
    if any(
        _point_in_polygon2(point, polygon_a, tolerance)
        for point in polygon_b
    ):
        return True
    return any(
        _segments_intersect2(
            point_a,
            point_b,
            point_c,
            point_d,
            tolerance,
        )
        for point_a, point_b in zip(
            polygon_a,
            polygon_a[1:] + polygon_a[:1],
        )
        for point_c, point_d in zip(
            polygon_b,
            polygon_b[1:] + polygon_b[:1],
        )
    )


def _corner_topology_reachable_faces(
    owner_face_id,
    owner_normal,
    faces,
    edges,
    rail_edge_ids,
    dam_vertex_ids,
):
    """Coplanar область угла до геометрического clip, с RR barriers."""

    reachable = {owner_face_id}
    pending = [owner_face_id]
    while pending:
        face_id = pending.pop()
        for edge_id in faces[face_id].edge_ids:
            edge = edges[edge_id]
            if (
                edge.is_pchain
                or edge.is_spine
                or edge_id in rail_edge_ids
                or any(vertex_id in dam_vertex_ids for vertex_id in edge.vertex_ids)
            ):
                continue
            for neighbor_id in sorted(edge.face_indices, reverse=True):
                if neighbor_id in reachable or neighbor_id not in faces:
                    continue
                if _dot3(faces[neighbor_id].normal, owner_normal) < _RAIL_PLANAR_DOT:
                    continue
                reachable.add(neighbor_id)
                pending.append(neighbor_id)
    return frozenset(reachable)


def _corner_compile_vertex(
    *,
    partition,
    normal,
    face_id,
    path_by_id,
    station_lookup,
    position,
    key,
    source_feature,
    source_feature_id,
    source_edge_id=None,
    path_hint=None,
):
    matches = station_lookup.get(position, ())
    if position == partition.point:
        key = ("rail-source-vertex", partition.source_vertex_id)
        source_feature = "SOURCE_VERTEX"
        source_feature_id = partition.source_vertex_id
        path_id = ("CORNER", partition.partition_id)
        station_index = 0
    elif matches:
        matching_paths = {
            matched_path_id for matched_path_id, _vertex in matches
        }
        if path_hint in matching_paths:
            matched_path_id, boundary_vertex = next(
                item for item in matches if item[0] == path_hint
            )
        elif len(matching_paths) == 1:
            matched_path_id, boundary_vertex = matches[0]
        else:
            raise _RailGeometryCompileError(
                "RAIL_GEOMETRY_CORNER_STATION_AMBIGUOUS",
                vertex_indices=(partition.source_vertex_id,),
                face_indices=(face_id,),
            )
        key = boundary_vertex.key
        source_feature = boundary_vertex.source_feature
        source_feature_id = boundary_vertex.source_feature_id
        source_edge_id = boundary_vertex.source_edge_id
        path_id = matched_path_id
        station_index = boundary_vertex.station_index
    else:
        path_id = path_hint or ("CORNER", partition.partition_id)
        station_index = None
    route_ids = tuple(
        sorted(
            {
                path_by_id[partition.previous_path_id].route_id,
                path_by_id[partition.next_path_id].route_id,
            }
        )
    )
    radius = _corner_radius(partition, position, normal)
    arithmetic = 1e-12 * max(1.0, abs(radius))
    if abs(radius) <= arithmetic:
        radius = 0.0
    return RailDomainVertex(
        key=key,
        position=position,
        s=partition.s,
        r=radius,
        boundary_path_id=path_id,
        source_feature=source_feature,
        source_feature_id=source_feature_id,
        source_edge_id=source_edge_id,
        source_face_id=face_id,
        route_ids=route_ids,
        station_index=station_index,
    )


def _clip_corner_polygon_by_line(
    polygon,
    *,
    partition,
    normal,
    face_id,
    path_by_id,
    station_lookup,
    line_start,
    line_end,
    interior_point,
    line_key,
    path_hint=None,
):
    line = _sub3(line_end, line_start)
    interior_value = _dot3(
        _cross3(line, _sub3(interior_point, line_start)),
        normal,
    )
    if interior_value == 0.0:
        raise _RailGeometryCompileError(
            "RAIL_GEOMETRY_CORNER_CLIP_LINE_INVALID",
            vertex_indices=(partition.source_vertex_id,),
            face_indices=(face_id,),
        )
    inside_sign = 1.0 if interior_value > 0.0 else -1.0

    def signed_value(vertex):
        value = inside_sign * _dot3(
            _cross3(line, _sub3(vertex.position, line_start)),
            normal,
        )
        arithmetic = 1e-12 * max(
            1.0,
            _length3(line),
            _length3(_sub3(vertex.position, partition.point)),
        )
        return 0.0 if abs(value) <= arithmetic else value

    output = []
    for index, current in enumerate(polygon):
        previous = polygon[index - 1]
        current_value = signed_value(current)
        previous_value = signed_value(previous)
        current_inside = current_value >= 0.0
        previous_inside = previous_value >= 0.0
        if current_inside != previous_inside:
            vertex_a, value_a = previous, previous_value
            vertex_b, value_b = current, current_value
            if repr(vertex_b.key) < repr(vertex_a.key):
                vertex_a, vertex_b = vertex_b, vertex_a
                value_a, value_b = value_b, value_a
            denominator = value_a - value_b
            if denominator == 0.0:
                raise _RailGeometryCompileError(
                    "RAIL_GEOMETRY_CORNER_CLIP_INTERSECTION_INVALID",
                    vertex_indices=(partition.source_vertex_id,),
                    face_indices=(face_id,),
                )
            parameter = value_a / denominator
            position = _add3(
                vertex_a.position,
                _mul3(
                    _sub3(vertex_b.position, vertex_a.position),
                    parameter,
                ),
            )
            shared_path = (
                vertex_a.boundary_path_id
                if vertex_a.boundary_path_id == vertex_b.boundary_path_id
                else None
            )
            intersection_path = path_hint or (
                shared_path
                if shared_path in {
                    partition.previous_path_id,
                    partition.next_path_id,
                }
                else None
            )
            output.append(
                _corner_compile_vertex(
                    partition=partition,
                    normal=normal,
                    face_id=face_id,
                    path_by_id=path_by_id,
                    station_lookup=station_lookup,
                    position=position,
                    key=(
                        "rail-corner-compile-cut",
                        partition.partition_id,
                        line_key,
                        tuple(
                            sorted(
                                (vertex_a.key, vertex_b.key),
                                key=repr,
                            )
                        ),
                    ),
                    source_feature="CORNER_COMPILE_CUT",
                    source_feature_id=(
                        partition.partition_id,
                        line_key,
                        vertex_a.key,
                        vertex_b.key,
                    ),
                    source_edge_id=(
                        vertex_a.source_edge_id
                        if vertex_a.source_edge_id == vertex_b.source_edge_id
                        else None
                    ),
                    path_hint=intersection_path,
                )
            )
        if current_inside:
            output.append(current)
    deduplicated = []
    for vertex in output:
        if deduplicated and deduplicated[-1].key == vertex.key:
            continue
        deduplicated.append(vertex)
    if deduplicated and deduplicated[0].key == deduplicated[-1].key:
        deduplicated.pop()
    return tuple(deduplicated)


def _compile_outer_corner_cells(
    rail_plan,
    component_id,
    partitions,
    path_by_id,
    alpha_budget,
):
    faces = _face_by_id(rail_plan)
    vertices = _position_by_vertex(rail_plan)
    edges = _edge_by_id(rail_plan)
    rail_edge_ids = {
        segment.edge_id
        for route in rail_plan.routes
        for segment in route.segments
    }
    dam_vertex_ids = {
        station.source_vertex_id
        for event in rail_plan.events
        if event.kind.value == "DAM"
        for route in rail_plan.routes
        if route.route_id == event.route_id
        for station in route.stations
        if station.station_index == event.station_index
        and station.kind == RailStationKind.VERTEX
    }
    cells = []
    for partition in partitions:
        if partition.mode != "OUTER_FILL" or partition.owner_face_id is None:
            continue
        owner_face = faces[partition.owner_face_id]
        normal = owner_face.normal
        maximum_alpha = min(alpha_budget, partition.alpha_limit)
        previous = _add3(
            partition.point,
            _mul3(partition.previous_lateral, maximum_alpha),
        )
        miter = _add3(
            partition.point,
            _mul3(partition.miter_vector, maximum_alpha),
        )
        next_point = _add3(
            partition.point,
            _mul3(partition.next_lateral, maximum_alpha),
        )
        maximum_window = (
            partition.point,
            previous,
            miter,
            next_point,
        )
        interior = _mul3(
            _add3(
                _add3(partition.point, previous),
                _add3(miter, next_point),
            ),
            0.25,
        )
        clip_lines = (
            (
                partition.point,
                previous,
                "PREVIOUS_SIDE",
                partition.previous_path_id,
            ),
            (previous, miter, "PREVIOUS_FRONTIER", None),
            (miter, next_point, "NEXT_FRONTIER", None),
            (
                next_point,
                partition.point,
                "NEXT_SIDE",
                partition.next_path_id,
            ),
        )
        station_lookup = _corner_path_station_lookup(path_by_id, partition)
        candidate_polygons = {}
        topology_reachable = _corner_topology_reachable_faces(
            partition.owner_face_id,
            normal,
            faces,
            edges,
            rail_edge_ids,
            dam_vertex_ids,
        )
        for face_id in sorted(topology_reachable):
            face = faces[face_id]
            if _dot3(face.normal, normal) < _RAIL_PLANAR_DOT:
                continue
            source_polygon = tuple(
                vertices[vertex_id] for vertex_id in face.vertex_ids
            )
            if not _source_face_is_convex(face, vertices):
                if _coplanar_polygons_overlap(
                    source_polygon,
                    maximum_window,
                    normal,
                ):
                    raise _RailGeometryCompileError(
                        "RAIL_GEOMETRY_SOURCE_FACE_NON_CONVEX",
                        edge_indices=face.edge_ids,
                        vertex_indices=face.vertex_ids,
                        face_indices=(face_id,),
                        details=(
                            ("corner_vertex_id", partition.source_vertex_id),
                            ("corner_partition_id", partition.partition_id),
                        ),
                    )
                continue
            polygon = tuple(
                _corner_compile_vertex(
                    partition=partition,
                    normal=normal,
                    face_id=face_id,
                    path_by_id=path_by_id,
                    station_lookup=station_lookup,
                    position=vertices[vertex_id],
                    key=("rail-source-vertex", vertex_id),
                    source_feature="SOURCE_VERTEX",
                    source_feature_id=vertex_id,
                )
                for vertex_id in face.vertex_ids
            )
            for line_start, line_end, line_key, path_hint in clip_lines:
                polygon = _clip_corner_polygon_by_line(
                    polygon,
                    partition=partition,
                    normal=normal,
                    face_id=face_id,
                    path_by_id=path_by_id,
                    station_lookup=station_lookup,
                    line_start=line_start,
                    line_end=line_end,
                    interior_point=interior,
                    line_key=line_key,
                    path_hint=path_hint,
                )
                if len(polygon) < 3:
                    break
            if len(polygon) < 3:
                continue
            scale = max(
                1.0,
                *(
                    _length3(_sub3(vertex.position, partition.point))
                    for vertex in polygon
                ),
            )
            if _polygon_area_measure(polygon) <= scale * scale * 1e-12:
                continue
            polygon_normal = _polygon_normal(polygon)
            if polygon_normal is None:
                continue
            if _dot3(polygon_normal, face.normal) < 0.0:
                polygon = tuple(reversed(polygon))
            divider_values = tuple(
                _divider_value(partition, vertex.position, normal)
                for vertex in polygon
            )
            if any(value > 0.0 for value in divider_values) and any(
                value < 0.0 for value in divider_values
            ):
                split_polygons = tuple(
                    _clip_by_corner_divider(
                        polygon,
                        partition,
                        normal,
                        keep_sign,
                        component_id,
                        face_id,
                    )
                    for keep_sign in (-1, 1)
                )
            else:
                split_polygons = (polygon,)
            valid_polygons = tuple(
                split_polygon
                for split_polygon in split_polygons
                if len(split_polygon) >= 3
                and _polygon_area_measure(split_polygon) > scale * scale * 1e-12
            )
            if valid_polygons:
                candidate_polygons[face_id] = valid_polygons

        if partition.owner_face_id not in candidate_polygons:
            raise _RailGeometryCompileError(
                "RAIL_GEOMETRY_CORNER_OWNER_CELL_MISSING",
                vertex_indices=(partition.source_vertex_id,),
                face_indices=(partition.owner_face_id,),
            )
        candidate_ids = set(candidate_polygons)
        reachable = {partition.owner_face_id}
        pending = [partition.owner_face_id]
        while pending:
            face_id = pending.pop()
            for edge_id in faces[face_id].edge_ids:
                edge = edges[edge_id]
                if (
                    edge.is_pchain
                    or edge.is_spine
                    or edge_id in rail_edge_ids
                    or any(
                        vertex_id in dam_vertex_ids
                        for vertex_id in edge.vertex_ids
                    )
                ):
                    continue
                for neighbor_id in edges[edge_id].face_indices:
                    if neighbor_id in candidate_ids and neighbor_id not in reachable:
                        reachable.add(neighbor_id)
                        pending.append(neighbor_id)
        for face_id in sorted(reachable):
            face = faces[face_id]
            for polygon in candidate_polygons[face_id]:
                cells.append(
                    RailCornerCell(
                        cell_id=len(cells),
                        partition_id=partition.partition_id,
                        owner_face_id=face_id,
                        spine_edge_id=partition.source_edge_ids[0],
                        source_edge_ids=tuple(
                            sorted(
                                {
                                    *partition.source_edge_ids,
                                    *face.edge_ids,
                                }
                            )
                        ),
                        boundary_path_ids=(
                            partition.previous_path_id,
                            partition.next_path_id,
                        ),
                        vertices=polygon,
                    )
                )
    return tuple(cells)


def _compile_planar_rail_geometry_plan(
    rail_plan,
    edge_indices=(),
    *,
    apex_limit=DECAL_CORNER_MITER_LIMIT,
    split_angle=pi / 3.0,
):
    component_edges = _select_component(rail_plan, edge_indices)
    component = _compile_spine_component(rail_plan, component_edges)
    component_vertices = set(component.vertex_ids)
    for route in rail_plan.routes:
        if route.key.side.spine_vertex_id not in component_vertices:
            continue
        if route.termination == RailTermination.MERGE:
            raise _RailGeometryCompileError(
                "RAIL_GEOMETRY_MERGE_UNSUPPORTED",
                edge_indices=component.edge_ids,
                vertex_indices=(route.key.side.spine_vertex_id,),
                route_ids=(route.route_id,),
            )
        if route.termination == RailTermination.POLE:
            raise _RailGeometryCompileError(
                "RAIL_GEOMETRY_POLE_UNSUPPORTED",
                edge_indices=component.edge_ids,
                vertex_indices=(route.key.side.spine_vertex_id,),
                route_ids=(route.route_id,),
            )
    faces = _face_by_id(rail_plan)
    trace_cache = {}
    cap_traces = []
    channels = []
    path_by_id = {}
    next_cell_id = 0
    for interval in component.intervals:
        spine_edge = _edge_by_id(rail_plan)[interval.source_edge_id]
        for face_id in interval.source_face_ids:
            face = faces[face_id]
            if face.planarity_min_dot < _RAIL_PLANAR_DOT:
                raise _RailGeometryCompileError(
                    "RAIL_GEOMETRY_SOURCE_FACE_NON_PLANAR",
                    edge_indices=(interval.source_edge_id,),
                    face_indices=(face_id,),
                )
            path_a = _path_for_endpoint(
                rail_plan,
                component,
                interval,
                face_id,
                interval.from_station_index,
                trace_cache,
                cap_traces,
            )
            path_b = _path_for_endpoint(
                rail_plan,
                component,
                interval,
                face_id,
                interval.to_station_index,
                trace_cache,
                cap_traces,
            )
            path_by_id[path_a.path_id] = path_a
            path_by_id[path_b.path_id] = path_b
            cells = _compile_channel_cells(
                rail_plan=rail_plan,
                channel_id=len(channels),
                component=component,
                interval=interval,
                initial_face_id=face_id,
                path_a=path_a,
                path_b=path_b,
                first_cell_id=next_cell_id,
            )
            next_cell_id += len(cells)
            channels.append(
                RailChannel(
                    channel_id=len(channels),
                    spine_edge_id=interval.source_edge_id,
                    initial_face_id=face_id,
                    side_sign=_side_sign(
                        face,
                        interval.source_edge_id,
                        component.vertex_ids[interval.from_station_index],
                        component.vertex_ids[interval.to_station_index],
                    ),
                    from_path_id=path_a.path_id,
                    to_path_id=path_b.path_id,
                    cells=cells,
                )
            )

    by_vertex_edges = defaultdict(list)
    edge_by_id = _edge_by_id(rail_plan)
    for edge_id in component.edge_ids:
        for vertex_id in edge_by_id[edge_id].vertex_ids:
            by_vertex_edges[vertex_id].append(edge_id)
    corners = tuple(
        RailCornerUse(
            source_vertex_id=vertex_id,
            incident_spine_edge_ids=tuple(sorted(edge_ids)),
            source_face_ids=tuple(
                sorted(
                    set(edge_by_id[edge_ids[0]].face_indices).union(
                        edge_by_id[edge_ids[1]].face_indices
                    )
                )
            ),
        )
        for vertex_id, edge_ids in sorted(by_vertex_edges.items())
        if len(edge_ids) == 2
    )
    corner_partitions = _compile_corner_partitions(
        rail_plan,
        component,
        channels,
        path_by_id,
        apex_limit,
        split_angle,
    )
    path_scales = _compile_path_reach_scales(
        path_by_id,
        corner_partitions,
    )
    channels = _normalize_channels(channels, path_by_id, path_scales)
    channels, corner_partitions = _synchronize_corner_limits(
        channels,
        corner_partitions,
        path_by_id,
        path_scales,
    )
    effective_alpha_budget = _effective_alpha_budget(
        rail_plan,
        path_by_id,
        path_scales,
    )
    corner_cells = _compile_outer_corner_cells(
        rail_plan,
        component.component_id,
        corner_partitions,
        path_by_id,
        effective_alpha_budget,
    )
    return PlanarRailGeometryPlan(
        rail_plan=rail_plan,
        component=component,
        channels=tuple(channels),
        cap_traces=tuple(cap_traces),
        corners=corners,
        corner_partitions=corner_partitions,
        corner_cells=corner_cells,
        boundary_paths=tuple(
            path_by_id[path_id] for path_id in sorted(path_by_id, key=repr)
        ),
        path_reach_scales=tuple(
            (path_id, path_scales[path_id])
            for path_id in sorted(path_scales, key=repr)
        ),
        alpha_budget=effective_alpha_budget,
        apex_limit=float(apex_limit),
        split_angle=float(split_angle),
    )


def compile_planar_rail_geometry_attempt(
    rail_plan,
    *,
    edge_indices=(),
    apex_limit=DECAL_CORNER_MITER_LIMIT,
    split_angle=pi / 3.0,
    dynamic_corner_bands=False,
):
    """Compile-only PLANAR admission одного физического spine-компонента."""

    try:
        if dynamic_corner_bands:
            raise _RailGeometryCompileError(
                "RAIL_GEOMETRY_DYNAMIC_BANDS_UNSUPPORTED",
                edge_indices=edge_indices,
            )
        apex_limit = float(apex_limit)
        if not apex_limit > 0.0:
            raise _RailGeometryCompileError(
                "RAIL_GEOMETRY_APEX_LIMIT_INVALID",
                edge_indices=edge_indices,
            )
        split_angle = float(split_angle)
        if split_angle < 0.0 or split_angle > pi:
            raise _RailGeometryCompileError(
                "RAIL_GEOMETRY_SPLIT_ANGLE_INVALID",
                edge_indices=edge_indices,
            )
        plan = _compile_planar_rail_geometry_plan(
            rail_plan,
            edge_indices,
            apex_limit=apex_limit,
            split_angle=split_angle,
        )
    except _RailGeometryCompileError as exc:
        return RailGeometryCompileAttempt(None, (exc.failure,))
    except Exception as exc:
        return RailGeometryCompileAttempt(
            None,
            (
                RailGeometryFailure(
                    reason="RAIL_GEOMETRY_COMPILE_INTERNAL_ERROR",
                    details=(
                        ("exception_type", type(exc).__name__),
                        ("message", str(exc)),
                    ),
                ),
            ),
        )
    return RailGeometryCompileAttempt(plan, ())


def _interpolate_domain_vertex(vertex_a, vertex_b, alpha, component_id):
    if repr(vertex_b.key) < repr(vertex_a.key):
        vertex_a, vertex_b = vertex_b, vertex_a
    if alpha == vertex_a.r:
        return vertex_a
    if alpha == vertex_b.r:
        return vertex_b
    denominator = vertex_b.r - vertex_a.r
    if denominator == 0.0:
        raise RailGeometryEvaluationError(
            RailGeometryFailure(
                reason="RAIL_GEOMETRY_FRONTIER_INTERSECTION_INVALID",
                face_indices=(vertex_a.source_face_id,),
            )
        )
    parameter = (alpha - vertex_a.r) / denominator
    position = _add3(
        vertex_a.position,
        _mul3(_sub3(vertex_b.position, vertex_a.position), parameter),
    )
    s = vertex_a.s + (vertex_b.s - vertex_a.s) * parameter
    shared_path_id = None
    if vertex_a.boundary_path_id == vertex_b.boundary_path_id:
        shared_path_id = vertex_a.boundary_path_id
    elif (
        vertex_a.boundary_path_id[:1] == ("CORNER",)
        and vertex_b.boundary_path_id[:1] != ("CORNER",)
    ):
        shared_path_id = vertex_b.boundary_path_id
    elif (
        vertex_b.boundary_path_id[:1] == ("CORNER",)
        and vertex_a.boundary_path_id[:1] != ("CORNER",)
    ):
        shared_path_id = vertex_a.boundary_path_id
    if shared_path_id is not None:
        if shared_path_id[:1] == ("CORNER",):
            ordered_keys = tuple(
                sorted((vertex_a.key, vertex_b.key), key=repr)
            )
            key = (
                "rail-corner-frontier",
                component_id,
                shared_path_id[1],
                ordered_keys,
                alpha,
            )
        else:
            key = (
                "rail-frontier",
                component_id,
                shared_path_id,
                alpha,
            )
        path_id = shared_path_id
    else:
        ordered_keys = tuple(sorted((vertex_a.key, vertex_b.key), key=repr))
        key = ("rail-frontier-cell", component_id, ordered_keys, alpha)
        path_id = ("CELL", ordered_keys)
    return RailDomainVertex(
        key=key,
        position=position,
        s=s,
        r=alpha,
        boundary_path_id=path_id,
        source_feature="RAIL_FRONTIER",
        source_feature_id=(vertex_a.key, vertex_b.key, alpha),
        source_edge_id=(
            vertex_a.source_edge_id
            if vertex_a.source_edge_id == vertex_b.source_edge_id
            else None
        ),
        source_face_id=vertex_a.source_face_id,
        route_ids=tuple(sorted(set(vertex_a.route_ids).union(vertex_b.route_ids))),
        station_index=None,
    )


def _clip_cell(cell, alpha, component_id):
    output = []
    vertices = cell.vertices
    for index, current in enumerate(vertices):
        previous = vertices[index - 1]
        current_inside = current.r <= alpha
        previous_inside = previous.r <= alpha
        if current_inside != previous_inside:
            output.append(
                _interpolate_domain_vertex(
                    previous,
                    current,
                    alpha,
                    component_id,
                )
            )
        if current_inside:
            output.append(current)
    deduplicated = []
    for vertex in output:
        if deduplicated and deduplicated[-1].key == vertex.key:
            continue
        deduplicated.append(vertex)
    if (
        len(deduplicated) > 1
        and deduplicated[0].key == deduplicated[-1].key
    ):
        deduplicated.pop()
    return tuple(deduplicated)


def _polygon_area_measure(vertices):
    if len(vertices) < 3:
        return 0.0
    origin = vertices[0].position
    area_vector = (0.0, 0.0, 0.0)
    for index in range(1, len(vertices) - 1):
        area_vector = _add3(
            area_vector,
            _cross3(
                _sub3(vertices[index].position, origin),
                _sub3(vertices[index + 1].position, origin),
            ),
        )
    return _length3(area_vector) * 0.5


def _positive_clipped_extent(cell, clipped, alpha):
    if len(clipped) < 3:
        return False
    scale = max(
        1.0,
        *(
            _length3(_sub3(vertex.position, cell.vertices[0].position))
            for vertex in cell.vertices[1:]
        ),
    )
    area_limit = scale * scale * 1e-12
    area = _polygon_area_measure(clipped)
    if area > area_limit:
        return True
    minimum_r = min(vertex.r for vertex in cell.vertices)
    maximum_r = max(vertex.r for vertex in cell.vertices)
    radial_extent = min(alpha, maximum_r) - minimum_r
    if radial_extent <= scale * 1e-12:
        return False
    raise RailGeometryEvaluationError(
        RailGeometryFailure(
            reason="RAIL_GEOMETRY_CLIPPED_CELL_DEGENERATE",
            edge_indices=(cell.spine_edge_id,),
            face_indices=(cell.owner_face_id,),
            details=(
                ("cell_id", cell.cell_id),
                ("area", area),
                ("radial_extent", radial_extent),
            ),
        )
    )


def _divider_value(partition, point, normal):
    return _dot3(
        _cross3(partition.miter_vector, _sub3(point, partition.point)),
        normal,
    )


def _divider_intersection(
    vertex_a,
    vertex_b,
    value_a,
    value_b,
    partition,
    component_id,
    face_id=None,
):
    if repr(vertex_b.key) < repr(vertex_a.key):
        vertex_a, vertex_b = vertex_b, vertex_a
        value_a, value_b = value_b, value_a
    denominator = value_a - value_b
    if denominator == 0.0:
        raise RailGeometryEvaluationError(
            RailGeometryFailure(
                reason="RAIL_GEOMETRY_CORNER_DIVIDER_INVALID",
                vertex_indices=(partition.source_vertex_id,),
                face_indices=(partition.owner_face_id,),
            )
        )
    parameter = value_a / denominator
    if parameter == 0.0:
        return vertex_a
    if parameter == 1.0:
        return vertex_b
    r = vertex_a.r + (vertex_b.r - vertex_a.r) * parameter
    if vertex_a.r == vertex_b.r:
        # Один semantic miter-key может прийти с двух соседних cell edges.
        # Считаем его из единственного compile-frame, а не двумя float lerp.
        position = _add3(
            partition.point,
            _mul3(partition.miter_vector, r),
        )
        s = partition.s
        key = (
            "rail-corner-miter",
            component_id,
            partition.partition_id,
            r,
        )
    else:
        position = _add3(
            vertex_a.position,
            _mul3(_sub3(vertex_b.position, vertex_a.position), parameter),
        )
        s = vertex_a.s + (vertex_b.s - vertex_a.s) * parameter
        key = (
            "rail-corner-divider",
            component_id,
            partition.partition_id,
            tuple(sorted((vertex_a.key, vertex_b.key), key=repr)),
        )
    return RailDomainVertex(
        key=key,
        position=position,
        s=s,
        r=r,
        boundary_path_id=("CORNER", partition.partition_id),
        source_feature="CORNER_DIVIDER",
        source_feature_id=(partition.partition_id, vertex_a.key, vertex_b.key),
        source_edge_id=None,
        source_face_id=(
            partition.owner_face_id if face_id is None else face_id
        ),
        route_ids=tuple(sorted(set(vertex_a.route_ids).union(vertex_b.route_ids))),
        station_index=None,
    )


def _clip_by_corner_divider(
    vertices,
    partition,
    normal,
    keep_sign,
    component_id,
    face_id=None,
):
    output = []
    for index, current in enumerate(vertices):
        previous = vertices[index - 1]
        current_value = _divider_value(partition, current.position, normal)
        previous_value = _divider_value(partition, previous.position, normal)
        current_inside = current_value * keep_sign >= 0.0
        previous_inside = previous_value * keep_sign >= 0.0
        if current_inside != previous_inside:
            output.append(
                _divider_intersection(
                    previous,
                    current,
                    previous_value,
                    current_value,
                    partition,
                    component_id,
                    face_id,
                )
            )
        if current_inside:
            output.append(current)
    deduplicated = []
    for vertex in output:
        if deduplicated and deduplicated[-1].key == vertex.key:
            continue
        deduplicated.append(vertex)
    if deduplicated and deduplicated[0].key == deduplicated[-1].key:
        deduplicated.pop()
    return tuple(deduplicated)


def _ordered_path_vertices(path):
    by_index = defaultdict(set)
    for piece in path.pieces:
        by_index[piece.piece_index].add((piece.start, piece.end))
    ordered = []
    for piece_index in sorted(by_index):
        candidates = by_index[piece_index]
        if len(candidates) != 1:
            raise RailGeometryEvaluationError(
                RailGeometryFailure(
                    reason="RAIL_GEOMETRY_BOUNDARY_PATH_DESYNC",
                    route_ids=(path.route_id,),
                    details=(("piece_index", piece_index),),
                )
            )
        start, end = next(iter(candidates))
        if not ordered:
            ordered.append(start)
        elif ordered[-1].key != start.key:
            raise RailGeometryEvaluationError(
                RailGeometryFailure(
                    reason="RAIL_GEOMETRY_BOUNDARY_PATH_DISCONTINUOUS",
                    route_ids=(path.route_id,),
                    details=(("piece_index", piece_index),),
                )
            )
        ordered.append(end)
    return tuple(ordered)


def evaluate_planar_rail_geometry_plan(
    plan,
    width,
    offset=0.0,
    preview=False,
):
    """Clips immutable R1 cells; ``preview`` cannot change geometry."""

    del preview
    width = float(width)
    if not width > 0.0:
        raise RailGeometryEvaluationError(
            RailGeometryFailure(reason="RAIL_GEOMETRY_WIDTH_INVALID")
        )
    alpha = width * 0.5
    if alpha > plan.alpha_budget:
        raise RailGeometryEvaluationError(
            RailGeometryFailure(
                reason="RAIL_GEOMETRY_DOMAIN_BUDGET_EXCEEDED",
                edge_indices=plan.component.edge_ids,
                details=(
                    ("requested_alpha", alpha),
                    ("alpha_budget", plan.alpha_budget),
                ),
            )
        )
    faces_by_id = _face_by_id(plan.rail_plan)
    pending = []
    for channel in plan.channels:
        channel_alpha = min(alpha, channel.alpha_limit)
        for cell in channel.cells:
            clipped = _clip_cell(
                cell,
                channel_alpha,
                plan.component.component_id,
            )
            if not _positive_clipped_extent(cell, clipped, channel_alpha):
                continue
            pending.append([channel, cell, clipped, channel_alpha])

    for partition in plan.corner_partitions:
        if partition.mode != "INNER_DIVIDER":
            continue
        owner_face = faces_by_id[partition.owner_face_id]
        for channel_id in (
            partition.previous_channel_id,
            partition.next_channel_id,
        ):
            records = [
                record
                for record in pending
                if record[0].channel_id == channel_id
                and record[1].owner_face_id == partition.owner_face_id
            ]
            if len(records) != 1:
                raise RailGeometryEvaluationError(
                    RailGeometryFailure(
                        reason="RAIL_GEOMETRY_CORNER_CELL_MISSING",
                        vertex_indices=(partition.source_vertex_id,),
                        face_indices=(partition.owner_face_id,),
                        details=(("channel_id", channel_id),),
                    )
                )
            record = records[0]
            spine_vertices = [
                vertex
                for vertex in record[2]
                if vertex.r == 0.0
                and vertex.key
                != ("rail-source-vertex", partition.source_vertex_id)
            ]
            if len(spine_vertices) != 1:
                raise RailGeometryEvaluationError(
                    RailGeometryFailure(
                        reason="RAIL_GEOMETRY_CORNER_OWNER_UNRESOLVED",
                        vertex_indices=(partition.source_vertex_id,),
                        face_indices=(partition.owner_face_id,),
                        details=(("channel_id", channel_id),),
                    )
                )
            owner_value = _divider_value(
                partition,
                spine_vertices[0].position,
                owner_face.normal,
            )
            if owner_value == 0.0:
                raise RailGeometryEvaluationError(
                    RailGeometryFailure(
                        reason="RAIL_GEOMETRY_CORNER_OWNER_UNRESOLVED",
                        vertex_indices=(partition.source_vertex_id,),
                        face_indices=(partition.owner_face_id,),
                        details=(("channel_id", channel_id),),
                    )
                )
            divided = _clip_by_corner_divider(
                record[2],
                partition,
                owner_face.normal,
                1 if owner_value > 0.0 else -1,
                plan.component.component_id,
            )
            if not _positive_clipped_extent(record[1], divided, record[3]):
                raise RailGeometryEvaluationError(
                    RailGeometryFailure(
                        reason="RAIL_GEOMETRY_CORNER_PARTITION_EMPTY",
                        vertex_indices=(partition.source_vertex_id,),
                        face_indices=(partition.owner_face_id,),
                        details=(("channel_id", channel_id),),
                    )
                )
            record[2] = divided

    output = []
    for channel, cell, clipped, _channel_alpha in pending:
            owner_face = faces_by_id[cell.owner_face_id]
            normal = Vector(owner_face.normal)
            offset_vector = normal * float(offset)
            # Временный adapter: R4 заменит эту запись единственным
            # каноническим отображением (s, r) -> (U, V).
            u_fracs = [channel.side_sign * vertex.r / alpha for vertex in clipped]
            v_lengths = [vertex.s for vertex in clipped]
            provenance = RailFaceProvenance(
                component_id=plan.component.component_id,
                channel_id=channel.channel_id,
                cell_id=cell.cell_id,
                corner_partition_id=None,
                source_face_id=cell.owner_face_id,
                source_edge_ids=cell.source_edge_ids,
                boundary_path_ids=cell.boundary_path_ids,
                route_ids=tuple(
                    sorted(
                        {
                            route_id
                            for vertex in clipped
                            for route_id in vertex.route_ids
                        }
                    )
                ),
                station_keys=tuple(vertex.key for vertex in clipped),
            )
            output.append(
                RailDecalGeometryFace(
                    surface_id=cell.owner_face_id,
                    surface_normal=normal,
                    vert_keys=[vertex.key for vertex in clipped],
                    positions=[Vector(vertex.position) + offset_vector for vertex in clipped],
                    u_fracs=u_fracs,
                    v_lengths=v_lengths,
                    component_kind="RAIL_SEGMENT",
                    component_side=(
                        f"component:{plan.component.component_id}:"
                        f"channel:{channel.channel_id}:face:{cell.owner_face_id}"
                    ),
                    rail_provenance=provenance,
                )
            )

    partitions_by_id = {
        partition.partition_id: partition
        for partition in plan.corner_partitions
    }
    for cell in plan.corner_cells:
        partition = partitions_by_id[cell.partition_id]
        corner_alpha = min(alpha, partition.alpha_limit)
        vertices = _clip_cell(
            cell,
            corner_alpha,
            plan.component.component_id,
        )
        if not _positive_clipped_extent(cell, vertices, corner_alpha):
            continue
        owner_face = faces_by_id[cell.owner_face_id]
        route_ids = tuple(
            sorted(
                {
                    route_id
                    for vertex in vertices
                    for route_id in vertex.route_ids
                }
            )
        )
        provenance = RailFaceProvenance(
            component_id=plan.component.component_id,
            channel_id=None,
            cell_id=cell.cell_id,
            corner_partition_id=partition.partition_id,
            source_face_id=cell.owner_face_id,
            source_edge_ids=cell.source_edge_ids,
            boundary_path_ids=cell.boundary_path_ids,
            route_ids=route_ids,
            station_keys=tuple(vertex.key for vertex in vertices),
        )
        normal = Vector(owner_face.normal)
        offset_vector = normal * float(offset)
        output.append(
            RailDecalGeometryFace(
                surface_id=cell.owner_face_id,
                surface_normal=normal,
                vert_keys=[vertex.key for vertex in vertices],
                positions=[Vector(vertex.position) + offset_vector for vertex in vertices],
                u_fracs=[partition.side_sign * vertex.r / alpha for vertex in vertices],
                v_lengths=[vertex.s for vertex in vertices],
                component_kind="RAIL_CORNER",
                component_side=(
                    f"component:{plan.component.component_id}:"
                    f"corner:{partition.partition_id}:face:{cell.owner_face_id}"
                ),
                rail_provenance=provenance,
            )
        )
    return tuple(output)


__all__ = (
    "PlanarRailGeometryPlan",
    "RailBoundaryPath",
    "RailBoundaryPiece",
    "RailBoundaryVertex",
    "RailCapTrace",
    "RailChannel",
    "RailChannelCell",
    "RailCornerCell",
    "RailCornerPartition",
    "RailCornerUse",
    "RailDecalGeometryFace",
    "RailDomainVertex",
    "RailFaceProvenance",
    "RailGeometryCompileAttempt",
    "RailGeometryEvaluationError",
    "RailGeometryFailure",
    "RailSpineComponent",
    "RailSpineInterval",
    "RailSpineStation",
    "compile_planar_rail_geometry_attempt",
    "evaluate_planar_rail_geometry_plan",
)
