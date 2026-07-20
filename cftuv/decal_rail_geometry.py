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

from .constants import DECAL_COPLANAR_DOT, DECAL_CORNER_MITER_LIMIT
from .decal_geometry import DecalGeometryFace, lift_offset_position
from .decal_rails import (
    RailStartSectorKind,
    RailStationKind,
    RailTerminalKind,
    RailTermination,
)


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
            route_ids=tuple(
                sorted(
                    {
                        int(value)
                        for value in route_ids
                        if value is not None
                    }
                )
            ),
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
    route_id: int | None
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
    route_id: int | None
    spine_vertex_id: int
    spine_edge_id: int
    initial_face_id: int
    direction: tuple[float, float, float]
    pieces: tuple[RailBoundaryPiece, ...]
    termination: str


@dataclass(frozen=True)
class RailBoundaryPath:
    path_id: tuple
    route_id: int | None
    spine_vertex_id: int
    kind: str
    pieces: tuple[RailBoundaryPiece, ...]
    termination: str
    cap_trace_id: int | None = None
    corner_sector_id: object | None = None
    corner_side: int | None = None
    owner_face_ids: tuple[int, ...] = ()


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
class RailAngularPiece:
    """Один source-face кусок RM7a-веера между двумя rail paths."""

    piece_id: int
    angular_channel_id: int
    owner_face_id: int
    spine_edge_id: int
    sector_id: int
    source_edge_ids: tuple[int, ...]
    boundary_path_ids: tuple[tuple, tuple]
    vertices: tuple[RailDomainVertex, ...]


@dataclass(frozen=True)
class RailAngularChannel:
    """RM7a-веер с единым apex-s и независимыми r двух берегов."""

    angular_channel_id: int
    sector_id: int
    spine_vertex_id: int
    spine_edge_id: int
    side_sign: int
    apex_s: float
    ordinary_path_id: tuple
    boundary_path_id: tuple
    pieces: tuple[RailAngularPiece, ...]
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
    owner_face_ids: tuple[int, ...]
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
    angular_channel_id: int | None = None
    sector_id: int | None = None


@dataclass
class RailDecalGeometryFace(DecalGeometryFace):
    """Materializer-compatible face with immutable rail provenance payload."""

    rail_provenance: RailFaceProvenance | None = None


@dataclass(frozen=True)
class PlanarRailGeometryPlan:
    rail_plan: object
    component: RailSpineComponent
    channels: tuple[RailChannel, ...]
    angular_channels: tuple[RailAngularChannel, ...]
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


def _corner_sector_for_endpoint(
    rail_plan,
    spine_vertex_id,
    spine_edge_id,
    face_id,
):
    candidates = tuple(
        sector
        for sector in rail_plan.start_sectors
        if sector.kind == RailStartSectorKind.CORNER
        and sector.spine_vertex_id == spine_vertex_id
        and spine_edge_id in sector.delimiter_edge_ids
        and face_id in sector.source_face_ids
    )
    if len(candidates) > 1:
        raise _RailGeometryCompileError(
            "RAIL_GEOMETRY_CORNER_SECTOR_AMBIGUOUS",
            edge_indices=(spine_edge_id,),
            vertex_indices=(spine_vertex_id,),
            face_indices=(face_id,),
            details=(
                ("sector_ids", tuple(sector.sector_id for sector in candidates)),
            ),
        )
    return candidates[0] if candidates else None


def _clip_ray_to_convex_face(
    face,
    origin,
    direction,
    alpha_budget,
    positions,
    corner_vertex_id,
):
    """Half-space пересечение ray с одной convex sector-face."""

    if not _source_face_is_convex(face, positions):
        raise _RailGeometryCompileError(
            "RAIL_GEOMETRY_SOURCE_FACE_NON_CONVEX",
            edge_indices=face.edge_ids,
            vertex_indices=face.vertex_ids,
            face_indices=(face.face_id,),
            details=(("corner_vertex_id", corner_vertex_id),),
        )
    lower = 0.0
    upper = float(alpha_budget)
    for vertex_a_id, vertex_b_id in zip(
        face.vertex_ids,
        face.vertex_ids[1:] + face.vertex_ids[:1],
    ):
        point_a = positions[vertex_a_id]
        point_b = positions[vertex_b_id]
        edge_vector = _sub3(point_b, point_a)
        value = _dot3(
            _cross3(edge_vector, _sub3(origin, point_a)),
            face.normal,
        )
        slope = _dot3(_cross3(edge_vector, direction), face.normal)
        if slope == 0.0:
            if value < 0.0:
                return None
            continue
        boundary = -value / slope
        if slope > 0.0:
            lower = max(lower, boundary)
        else:
            upper = min(upper, boundary)
        if upper < lower:
            return None
    return max(0.0, lower), min(float(alpha_budget), upper)


def _clip_ray_to_convex_points(
    points,
    normal,
    origin,
    direction,
    alpha_budget,
):
    lower = 0.0
    upper = float(alpha_budget)
    for point_a, point_b in zip(points, points[1:] + points[:1]):
        edge_vector = _sub3(point_b, point_a)
        value = _dot3(
            _cross3(edge_vector, _sub3(origin, point_a)),
            normal,
        )
        slope = _dot3(_cross3(edge_vector, direction), normal)
        if slope == 0.0:
            if value < 0.0:
                return None
            continue
        boundary = -value / slope
        if slope > 0.0:
            lower = max(lower, boundary)
        else:
            upper = min(upper, boundary)
        if upper < lower:
            return None
    return max(0.0, lower), min(float(alpha_budget), upper)


def _corner_boundary_path(
    rail_plan,
    component,
    interval,
    endpoint_station_index,
    initial_face_id,
    side_sign,
    sector,
):
    """RM6a: side-owned perpendicular, клиппированный union сектора."""

    faces = _face_by_id(rail_plan)
    edges = _edge_by_id(rail_plan)
    positions = _position_by_vertex(rail_plan)
    station = component.stations[endpoint_station_index]
    corner_vertex_id = station.source_vertex_id
    spine_edge = edges[interval.source_edge_id]
    base_face = faces[initial_face_id]
    bad_faces = tuple(
        sorted(
            face_id
            for face_id in sector.source_face_ids
            if _dot3(faces[face_id].normal, base_face.normal)
            < _RAIL_PLANAR_DOT
        )
    )
    if bad_faces:
        raise _RailGeometryCompileError(
            "RAIL_GEOMETRY_CORNER_SECTOR_NON_PLANAR",
            edge_indices=sector.delimiter_edge_ids,
            vertex_indices=(corner_vertex_id,),
            face_indices=bad_faces,
            details=(("sector_id", sector.sector_id),),
        )
    direction = _cap_direction(
        base_face,
        spine_edge,
        component.vertex_ids[interval.from_station_index],
        component.vertex_ids[interval.to_station_index],
        positions,
    )
    if direction is None:
        raise _RailGeometryCompileError(
            "RAIL_GEOMETRY_CORNER_PATH_DIRECTION_INVALID",
            edge_indices=(interval.source_edge_id,),
            vertex_indices=(corner_vertex_id,),
            face_indices=(initial_face_id,),
        )
    origin = positions[corner_vertex_id]
    intervals = []
    for face_id in sector.source_face_ids:
        clipped = _clip_ray_to_convex_face(
            faces[face_id],
            origin,
            direction,
            rail_plan.alpha_budget,
            positions,
            corner_vertex_id,
        )
        if clipped is None:
            continue
        lower, upper = clipped
        if upper > lower or (
            lower == 0.0
            and upper == 0.0
            and corner_vertex_id in faces[face_id].vertex_ids
        ):
            intervals.append((lower, upper, face_id))

    positive = sorted(
        (lower, upper)
        for lower, upper, _face_id in intervals
        if upper > lower
    )
    coverage = 0.0
    for lower, upper in positive:
        if lower > coverage:
            break
        coverage = max(coverage, upper)
        if coverage >= rail_plan.alpha_budget:
            coverage = float(rail_plan.alpha_budget)
            break
    if coverage <= 0.0:
        raise _RailGeometryCompileError(
            "RAIL_GEOMETRY_CORNER_PATH_MATTER_EMPTY",
            edge_indices=(interval.source_edge_id,),
            vertex_indices=(corner_vertex_id,),
            face_indices=sector.source_face_ids,
            details=(("sector_id", sector.sector_id),),
        )

    path_id = (
        "CORNER_PATH",
        corner_vertex_id,
        interval.source_edge_id,
        side_sign,
    )
    vertices_by_r = {}

    def boundary_vertex(radius):
        cached = vertices_by_r.get(radius)
        if cached is not None:
            return cached
        position = _add3(origin, _mul3(direction, radius))
        if radius == 0.0:
            key = ("rail-source-vertex", corner_vertex_id)
            feature = "SOURCE_VERTEX"
            feature_id = corner_vertex_id
        else:
            key = (
                "rail-corner-r",
                corner_vertex_id,
                interval.source_edge_id,
                side_sign,
                radius,
            )
            feature = "CORNER_PATH"
            feature_id = ("ANALYTIC_PATH", path_id, origin, direction)
        vertex = RailBoundaryVertex(
            key=key,
            position=position,
            r=float(radius),
            source_feature=feature,
            source_feature_id=feature_id,
            source_edge_id=None,
            route_id=None,
            station_index=None,
        )
        vertices_by_r[radius] = vertex
        return vertex

    pieces = []
    for lower, upper, face_id in sorted(
        intervals,
        key=lambda item: (item[0], item[1], item[2]),
    ):
        if lower > coverage:
            continue
        upper = min(upper, coverage)
        if upper < lower:
            continue
        pieces.append(
            RailBoundaryPiece(
                # Channel traversal не переходит из segment в corner-sector:
                # все face-attribution pieces читают одну геометрию path.
                piece_index=0,
                owner_face_id=face_id,
                start=boundary_vertex(lower),
                end=boundary_vertex(upper),
                source_edge_ids=(),
            )
        )
    return RailBoundaryPath(
        path_id=path_id,
        route_id=None,
        spine_vertex_id=corner_vertex_id,
        kind="CORNER",
        pieces=tuple(pieces),
        termination=(
            "ALPHA"
            if coverage >= rail_plan.alpha_budget
            else "SECTOR_BORDER"
        ),
        corner_sector_id=sector.sector_id,
        corner_side=side_sign,
    )


def _planar_region_face_ids(rail_plan, initial_face_id):
    """RP2-region: flood только через канонически копланарные non-barriers."""

    faces = _face_by_id(rail_plan)
    edges = _edge_by_id(rail_plan)
    owner_normal = faces[initial_face_id].normal
    region = {initial_face_id}
    pending = [initial_face_id]
    while pending:
        face_id = pending.pop()
        for edge_id in faces[face_id].edge_ids:
            edge = edges[edge_id]
            if edge.is_pchain or edge.is_spine or edge.is_fold:
                continue
            for neighbor_id in edge.face_indices:
                if neighbor_id == face_id or neighbor_id in region:
                    continue
                if (
                    _dot3(faces[neighbor_id].normal, owner_normal)
                    <= DECAL_COPLANAR_DOT
                ):
                    continue
                region.add(neighbor_id)
                pending.append(neighbor_id)
    return tuple(sorted(region))


def _in_plane_boundary_route(
    rail_plan,
    spine_vertex_id,
    region_face_ids,
):
    edges = _edge_by_id(rail_plan)
    region_face_ids = set(region_face_ids)
    candidates = tuple(
        route
        for route in rail_plan.routes
        if route.key.side.spine_vertex_id == spine_vertex_id
        and route.key.side.start_edge_id >= 0
        and edges[route.key.side.start_edge_id].is_pchain
        and region_face_ids.intersection(route.key.side.source_face_ids)
    )
    if len(candidates) > 1:
        raise _RailGeometryCompileError(
            "RAIL_GEOMETRY_IN_PLANE_BOUNDARY_AMBIGUOUS",
            vertex_indices=(spine_vertex_id,),
            face_indices=region_face_ids,
            route_ids=(route.route_id for route in candidates),
            details=(("candidate_count", len(candidates)),),
        )
    return candidates[0] if candidates else None


def _terminal_use_for_endpoint(
    rail_plan,
    spine_vertex_id,
    spine_edge_id,
    face_id,
):
    """RR9: materialization читает terminal choice, не выводит его заново."""

    candidates = tuple(
        use
        for use in rail_plan.terminal_uses
        if use.spine_vertex_id == spine_vertex_id
        and use.spine_edge_id == spine_edge_id
        and face_id in use.source_face_ids
    )
    if len(candidates) > 1:
        raise _RailGeometryCompileError(
            "RAIL_GEOMETRY_TERMINAL_USE_AMBIGUOUS",
            edge_indices=(spine_edge_id,),
            vertex_indices=(spine_vertex_id,),
            face_indices=(face_id,),
            route_ids=(
                use.route_id
                for use in candidates
                if use.route_id is not None
            ),
            details=(("candidate_count", len(candidates)),),
        )
    return candidates[0] if candidates else None


def _in_plane_boundary_path(
    rail_plan,
    component,
    interval,
    endpoint_station_index,
    initial_face_id,
    side_sign,
    region_face_ids,
):
    """RP1/RM6a: аналитический полуперпендикуляр через planar region."""

    faces = _face_by_id(rail_plan)
    edges = _edge_by_id(rail_plan)
    positions = _position_by_vertex(rail_plan)
    station = component.stations[endpoint_station_index]
    spine_vertex_id = station.source_vertex_id
    spine_edge = edges[interval.source_edge_id]
    base_face = faces[initial_face_id]
    direction = _cap_direction(
        base_face,
        spine_edge,
        component.vertex_ids[interval.from_station_index],
        component.vertex_ids[interval.to_station_index],
        positions,
    )
    if direction is None:
        raise _RailGeometryCompileError(
            "RAIL_GEOMETRY_IN_PLANE_PATH_DIRECTION_INVALID",
            edge_indices=(interval.source_edge_id,),
            vertex_indices=(spine_vertex_id,),
            face_indices=(initial_face_id,),
        )
    origin = positions[spine_vertex_id]
    intervals = []
    for face_id in region_face_ids:
        face = faces[face_id]
        source_points = tuple(
            positions[vertex_id] for vertex_id in face.vertex_ids
        )
        if _source_face_is_convex(face, positions):
            atoms = (source_points,)
        else:
            atoms = _triangulate_in_plane_source_polygon(
                source_points,
                face.normal,
            )
            if not atoms:
                raise _RailGeometryCompileError(
                    "RAIL_GEOMETRY_SOURCE_FACE_TRIANGULATION_INVALID",
                    edge_indices=face.edge_ids,
                    vertex_indices=face.vertex_ids,
                    face_indices=(face_id,),
                )
        for atom in atoms:
            clipped = _clip_ray_to_convex_points(
                atom,
                face.normal,
                origin,
                direction,
                rail_plan.alpha_budget,
            )
            if clipped is None:
                continue
            lower, upper = clipped
            if upper > lower or (
                lower == 0.0
                and upper == 0.0
                and spine_vertex_id in face.vertex_ids
            ):
                intervals.append((lower, upper, face_id))
    coverage = 0.0
    for lower, upper in sorted(
        (lower, upper)
        for lower, upper, _face_id in intervals
        if upper > lower
    ):
        if lower > coverage:
            break
        coverage = max(coverage, upper)
        if coverage >= rail_plan.alpha_budget:
            coverage = float(rail_plan.alpha_budget)
            break
    if coverage <= 0.0:
        raise _RailGeometryCompileError(
            "RAIL_GEOMETRY_IN_PLANE_PATH_MATTER_EMPTY",
            edge_indices=(interval.source_edge_id,),
            vertex_indices=(spine_vertex_id,),
            face_indices=region_face_ids,
        )
    internal_corner = sum(
        spine_vertex_id in edges[edge_id].vertex_ids
        for edge_id in component.edge_ids
    ) == 2
    incident_spine_edges = tuple(
        edge_id
        for edge_id in component.edge_ids
        if spine_vertex_id in edges[edge_id].vertex_ids
    )
    collinear_join = False
    if internal_corner:
        outward = []
        for edge_id in incident_spine_edges:
            other_vertex_id = next(
                vertex_id
                for vertex_id in edges[edge_id].vertex_ids
                if vertex_id != spine_vertex_id
            )
            outward.append(
                _normalized3(
                    _sub3(positions[other_vertex_id], origin)
                )
            )
        collinear_join = (
            None not in outward
            and _cross3(outward[0], outward[1]) == (0.0, 0.0, 0.0)
        )
    if collinear_join:
        kind = "IN_PLANE_JOIN"
        path_id = (
            "IN_PLANE_JOIN_PATH",
            spine_vertex_id,
            side_sign,
        )
    else:
        kind = "CORNER" if internal_corner else "IN_PLANE"
        path_id = (
            "CORNER_PATH" if internal_corner else "IN_PLANE_PATH",
            spine_vertex_id,
            interval.source_edge_id,
            side_sign,
        )
    vertices_by_r = {}

    def boundary_vertex(radius):
        cached = vertices_by_r.get(radius)
        if cached is not None:
            return cached
        position = _add3(origin, _mul3(direction, radius))
        if radius == 0.0:
            key = ("rail-source-vertex", spine_vertex_id)
            feature = "SOURCE_VERTEX"
            feature_id = spine_vertex_id
        else:
            key = (
                (
                    "rail-in-plane-join-r",
                    spine_vertex_id,
                    side_sign,
                    radius,
                )
                if collinear_join
                else (
                    "rail-in-plane-r",
                    spine_vertex_id,
                    interval.source_edge_id,
                    side_sign,
                    radius,
                )
            )
            feature = "IN_PLANE_PATH"
            feature_id = ("ANALYTIC_PATH", path_id, origin, direction)
        vertex = RailBoundaryVertex(
            key=key,
            position=position,
            r=float(radius),
            source_feature=feature,
            source_feature_id=feature_id,
            source_edge_id=None,
            route_id=None,
            station_index=None,
        )
        vertices_by_r[radius] = vertex
        return vertex

    pieces = tuple(
        RailBoundaryPiece(
            piece_index=0,
            owner_face_id=face_id,
            start=boundary_vertex(lower),
            end=boundary_vertex(min(upper, coverage)),
            source_edge_ids=(),
        )
        for lower, upper, face_id in sorted(
            intervals,
            key=lambda item: (item[0], item[1], item[2]),
        )
        if lower <= coverage and min(upper, coverage) >= lower
    )
    region_key = ("PLANAR_REGION", tuple(region_face_ids))
    return RailBoundaryPath(
        path_id=path_id,
        route_id=None,
        spine_vertex_id=spine_vertex_id,
        kind=kind,
        pieces=pieces,
        termination=(
            "ALPHA" if coverage >= rail_plan.alpha_budget else "REGION_BORDER"
        ),
        corner_sector_id=region_key if internal_corner else None,
        corner_side=side_sign if internal_corner else None,
        owner_face_ids=tuple(region_face_ids),
    )


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


def _dam_sector_for_endpoint(
    rail_plan,
    spine_vertex_id,
    spine_edge_id,
    face_id,
):
    candidates = tuple(
        sector
        for sector in rail_plan.start_sectors
        if sector.kind == RailStartSectorKind.DAM
        and sector.spine_vertex_id == spine_vertex_id
        and spine_edge_id in sector.delimiter_edge_ids
        and face_id in sector.source_face_ids
    )
    if len(candidates) > 1:
        raise _RailGeometryCompileError(
            "RAIL_GEOMETRY_STRUCTURAL_CAP_SECTOR_AMBIGUOUS",
            edge_indices=(spine_edge_id,),
            vertex_indices=(spine_vertex_id,),
            face_indices=(face_id,),
            details=(
                ("sector_ids", tuple(sector.sector_id for sector in candidates)),
            ),
        )
    return candidates[0] if candidates else None


def _trace_structural_cap(
    rail_plan,
    *,
    trace_id,
    route,
    spine_edge,
    initial_face_id,
    sector,
):
    """RM5a degradation: cap по смежному внутреннему ребру сектора."""

    edges = _edge_by_id(rail_plan)
    positions = _position_by_vertex(rail_plan)
    spine_vertex_id = route.key.side.spine_vertex_id
    candidates = tuple(
        edge_id
        for edge_id in sector.internal_edge_ids
        if initial_face_id in edges[edge_id].face_indices
        and spine_vertex_id in edges[edge_id].vertex_ids
    )
    if len(candidates) != 1:
        raise _RailGeometryCompileError(
            "RAIL_GEOMETRY_STRUCTURAL_CAP_EDGE_AMBIGUOUS",
            edge_indices=candidates,
            vertex_indices=(spine_vertex_id,),
            face_indices=(initial_face_id,),
            route_ids=(route.route_id,),
            details=(("candidate_count", len(candidates)),),
        )
    edge = edges[candidates[0]]
    other_vertex_id = (
        edge.vertex_ids[1]
        if edge.vertex_ids[0] == spine_vertex_id
        else edge.vertex_ids[0]
    )
    origin = positions[spine_vertex_id]
    end_position = positions[other_vertex_id]
    direction = _normalized3(_sub3(end_position, origin))
    if direction is None or not edge.length > 0.0:
        raise _RailGeometryCompileError(
            "RAIL_GEOMETRY_STRUCTURAL_CAP_EDGE_DEGENERATE",
            edge_indices=(edge.edge_id,),
            vertex_indices=edge.vertex_ids,
            face_indices=(initial_face_id,),
            route_ids=(route.route_id,),
        )
    start = RailBoundaryVertex(
        key=("rail-source-vertex", spine_vertex_id),
        position=origin,
        r=0.0,
        source_feature="SOURCE_VERTEX",
        source_feature_id=spine_vertex_id,
        source_edge_id=edge.edge_id,
        route_id=route.route_id,
        station_index=0,
    )
    end = RailBoundaryVertex(
        key=("rail-source-vertex", other_vertex_id),
        position=end_position,
        r=edge.length,
        source_feature="SOURCE_VERTEX",
        source_feature_id=other_vertex_id,
        source_edge_id=edge.edge_id,
        route_id=route.route_id,
        station_index=None,
    )
    piece = RailBoundaryPiece(
        piece_index=0,
        owner_face_id=initial_face_id,
        start=start,
        end=end,
        source_edge_ids=(edge.edge_id,),
    )
    trace = RailCapTrace(
        trace_id=trace_id,
        route_id=route.route_id,
        spine_vertex_id=spine_vertex_id,
        spine_edge_id=spine_edge.edge_id,
        initial_face_id=initial_face_id,
        direction=direction,
        pieces=(piece,),
        termination="STRUCTURAL",
    )
    return trace, RailBoundaryPath(
        path_id=("CAP", trace_id),
        route_id=route.route_id,
        spine_vertex_id=spine_vertex_id,
        kind="CAP",
        pieces=(piece,),
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
    face = _face_by_id(rail_plan)[face_id]
    side_sign = _side_sign(
        face,
        interval.source_edge_id,
        component.vertex_ids[interval.from_station_index],
        component.vertex_ids[interval.to_station_index],
    )
    region_face_ids = _planar_region_face_ids(rail_plan, face_id)
    terminal_use = _terminal_use_for_endpoint(
        rail_plan,
        station.source_vertex_id,
        interval.source_edge_id,
        face_id,
    )
    if terminal_use is not None:
        if terminal_use.kind == RailTerminalKind.ROUTE:
            routes = {
                route.route_id: route for route in rail_plan.routes
            }
            route = routes.get(terminal_use.route_id)
            if route is None:
                raise _RailGeometryCompileError(
                    "RAIL_GEOMETRY_TERMINAL_ROUTE_MISSING",
                    edge_indices=(interval.source_edge_id,),
                    vertex_indices=(station.source_vertex_id,),
                    face_indices=(face_id,),
                    route_ids=(terminal_use.route_id,),
                )
            cache_key = ("ROUTE", route.route_id)
            cached = trace_cache.get(cache_key)
            if cached is not None:
                return cached
            path = _route_boundary_path(rail_plan, route)
            trace_cache[cache_key] = path
            return path
        cache_key = (
            "IN_PLANE",
            station.source_vertex_id,
            interval.source_edge_id,
            side_sign,
        )
        cached = trace_cache.get(cache_key)
        if cached is not None:
            return cached
        path = _in_plane_boundary_path(
            rail_plan,
            component,
            interval,
            endpoint_station_index,
            face_id,
            side_sign,
            region_face_ids,
        )
        trace_cache[cache_key] = path
        return path

    corner_sector = _corner_sector_for_endpoint(
        rail_plan,
        station.source_vertex_id,
        interval.source_edge_id,
        face_id,
    )
    if corner_sector is not None:
        cache_key = (
            "CORNER",
            station.source_vertex_id,
            interval.source_edge_id,
            side_sign,
        )
        cached = trace_cache.get(cache_key)
        if cached is not None:
            return cached
        path = _corner_boundary_path(
            rail_plan,
            component,
            interval,
            endpoint_station_index,
            face_id,
            side_sign,
            corner_sector,
        )
        trace_cache[cache_key] = path
        return path
    boundary_route = _in_plane_boundary_route(
        rail_plan,
        station.source_vertex_id,
        region_face_ids,
    )
    if boundary_route is not None:
        cache_key = ("ROUTE", boundary_route.route_id)
        cached = trace_cache.get(cache_key)
        if cached is not None:
            return cached
        path = _route_boundary_path(rail_plan, boundary_route)
        trace_cache[cache_key] = path
        return path
    route_candidates = tuple(
        route
        for route in rail_plan.routes
        if route.key.side.spine_vertex_id == station.source_vertex_id
        and face_id in route.key.side.source_face_ids
        and route.key.side.start_edge_id >= 0
        and not _edge_by_id(rail_plan)[route.key.side.start_edge_id].is_pchain
    )
    if not route_candidates:
        cache_key = (
            "IN_PLANE",
            station.source_vertex_id,
            interval.source_edge_id,
            side_sign,
        )
        cached = trace_cache.get(cache_key)
        if cached is not None:
            return cached
        path = _in_plane_boundary_path(
            rail_plan,
            component,
            interval,
            endpoint_station_index,
            face_id,
            side_sign,
            region_face_ids,
        )
        trace_cache[cache_key] = path
        return path
    if len(route_candidates) != 1:
        raise _RailGeometryCompileError(
            "RAIL_GEOMETRY_BOUNDARY_AMBIGUOUS",
            vertex_indices=(station.source_vertex_id,),
            face_indices=(face_id,),
            route_ids=(route.route_id for route in route_candidates),
            details=(("candidate_count", len(route_candidates)),),
        )
    route = route_candidates[0]
    if route.key.side.start_edge_id >= 0:
        faces = _face_by_id(rail_plan)
        route_edge = _edge_by_id(rail_plan)[route.key.side.start_edge_id]
        if not route_edge.is_pchain:
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
        # RR8a: у boundary pChain один route и общие станции, но pieces
        # принадлежат face-sector'ам. Дальняя сторона fold не участвует в
        # текущем channel; фактическую кривизну материализуемой стороны ниже
        # проверяет _compile_channel_cells по owner faces самих cells.
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
        sector = _dam_sector_for_endpoint(
            rail_plan,
            station.source_vertex_id,
            interval.source_edge_id,
            face_id,
        )
        if sector is None:
            raise _RailGeometryCompileError(
                "RAIL_GEOMETRY_VIRTUAL_CAP_EMPTY",
                edge_indices=(interval.source_edge_id,),
                vertex_indices=(station.source_vertex_id,),
                face_indices=(face_id,),
                route_ids=(route.route_id,),
            )
        trace, path = _trace_structural_cap(
            rail_plan,
            trace_id=len(cap_traces),
            route=route,
            spine_edge=edge,
            initial_face_id=face_id,
            sector=sector,
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
        route_ids=(
            ()
            if boundary_vertex.route_id is None
            else (boundary_vertex.route_id,)
        ),
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


def _in_plane_station_lookup(*paths):
    lookup = defaultdict(list)
    for path in paths:
        for piece in path.pieces:
            for vertex in (piece.start, piece.end):
                lookup[vertex.position].append((path.path_id, vertex))
    return lookup


def _domain_source_vertex_id(vertex):
    if vertex.key[:1] == ("rail-source-vertex",):
        return int(vertex.key[1])
    return None


def _source_edge_for_domain_segment(
    vertex_a,
    vertex_b,
    *,
    edge_id_by_vertices,
    edges,
):
    """Возвращает физическое source-edge только при точном доказательстве."""

    if (
        vertex_a.source_edge_id is not None
        and vertex_a.source_edge_id == vertex_b.source_edge_id
    ):
        return vertex_a.source_edge_id

    source_vertex_a = _domain_source_vertex_id(vertex_a)
    source_vertex_b = _domain_source_vertex_id(vertex_b)
    if source_vertex_a is not None and source_vertex_b is not None:
        return edge_id_by_vertices.get(
            frozenset((source_vertex_a, source_vertex_b))
        )

    for edge_id, source_vertex_id in (
        (vertex_a.source_edge_id, source_vertex_b),
        (vertex_b.source_edge_id, source_vertex_a),
    ):
        if (
            edge_id is not None
            and source_vertex_id is not None
            and source_vertex_id in edges[edge_id].vertex_ids
        ):
            return edge_id
    return None


def _source_edge_station(edge_id, position, *, edges, positions):
    edge = edges[edge_id]
    point_a = positions[edge.vertex_ids[0]]
    point_b = positions[edge.vertex_ids[1]]
    edge_vector = _sub3(point_b, point_a)
    denominator = _dot3(edge_vector, edge_vector)
    parameter = (
        _dot3(_sub3(position, point_a), edge_vector) / denominator
        if denominator > 0.0
        else 0.0
        )
    if position == point_a:
        return (
            ("rail-source-vertex", edge.vertex_ids[0]),
            "SOURCE_VERTEX",
            edge.vertex_ids[0],
        )
    if position == point_b:
        return (
            ("rail-source-vertex", edge.vertex_ids[1]),
            "SOURCE_VERTEX",
            edge.vertex_ids[1],
        )
    return (
        ("rail-source-edge-station", edge_id, parameter),
        "SOURCE_EDGE",
        (edge_id, parameter),
    )


def _in_plane_domain_vertex(
    *,
    position,
    source_vertex_id,
    face_id,
    point_a,
    tangent,
    lateral,
    s_a,
    component_id,
    station_lookup,
):
    relative = _sub3(position, point_a)
    s = s_a + _dot3(relative, tangent)
    r = _dot3(relative, lateral)
    matches = station_lookup.get(position, ())
    matching_paths = {path_id for path_id, _vertex in matches}
    if source_vertex_id is not None:
        key = ("rail-source-vertex", source_vertex_id)
        feature = "SOURCE_VERTEX"
        feature_id = source_vertex_id
    else:
        key = (
            "rail-in-plane-point",
            component_id,
            tuple(float(value) for value in position),
        )
        feature = "IN_PLANE_POINT"
        feature_id = position
    if len(matching_paths) == 1:
        path_id, boundary_vertex = matches[0]
        r = boundary_vertex.r
        route_ids = (
            ()
            if boundary_vertex.route_id is None
            else (boundary_vertex.route_id,)
        )
        source_edge_id = boundary_vertex.source_edge_id
        station_index = boundary_vertex.station_index
        key = boundary_vertex.key
        feature = boundary_vertex.source_feature
        feature_id = boundary_vertex.source_feature_id
    else:
        path_id = (
            "IN_PLANE_SOURCE_VERTEX",
            component_id,
            source_vertex_id,
        )
        route_ids = ()
        source_edge_id = None
        station_index = None
    return RailDomainVertex(
        key=key,
        position=position,
        s=float(s),
        r=float(r),
        boundary_path_id=path_id,
        source_feature=feature,
        source_feature_id=feature_id,
        source_edge_id=source_edge_id,
        source_face_id=face_id,
        route_ids=route_ids,
        station_index=station_index,
    )


def _clip_in_plane_polygon(
    polygon,
    *,
    axis,
    bound,
    keep_greater,
    path_hint,
    constraint_key,
    component_id,
    station_lookup,
    edge_id_by_vertices,
    edges,
    positions,
):
    def value(vertex):
        coordinate = vertex.s if axis == "s" else vertex.r
        result = coordinate - bound
        return result if keep_greater else -result

    output = []
    for index, current in enumerate(polygon):
        previous = polygon[index - 1]
        current_value = value(current)
        previous_value = value(previous)
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
                    "RAIL_GEOMETRY_IN_PLANE_CLIP_INVALID",
                    face_indices=(current.source_face_id,),
                    details=(("constraint", constraint_key),),
                )
            parameter = value_a / denominator
            position = _add3(
                vertex_a.position,
                _mul3(_sub3(vertex_b.position, vertex_a.position), parameter),
            )
            s = vertex_a.s + (vertex_b.s - vertex_a.s) * parameter
            r = vertex_a.r + (vertex_b.r - vertex_a.r) * parameter
            if axis == "s":
                s = float(bound)
            else:
                r = float(bound)
            matches = station_lookup.get(position, ())
            matched = next(
                (
                    boundary_vertex
                    for matched_path_id, boundary_vertex in matches
                    if matched_path_id == path_hint
                ),
                None,
            )
            if matched is not None:
                key = matched.key
                path_id = path_hint
                source_feature = matched.source_feature
                source_feature_id = matched.source_feature_id
                source_edge_id = matched.source_edge_id
                station_index = matched.station_index
                route_ids = (
                    () if matched.route_id is None else (matched.route_id,)
                )
                r = matched.r
            else:
                source_edge_id = _source_edge_for_domain_segment(
                    vertex_a,
                    vertex_b,
                    edge_id_by_vertices=edge_id_by_vertices,
                    edges=edges,
                )
                shared_path_id = (
                    vertex_a.boundary_path_id
                    if vertex_a.boundary_path_id == vertex_b.boundary_path_id
                    else None
                )
                path_id = path_hint or shared_path_id or (
                    "IN_PLANE_CUT",
                    component_id,
                    constraint_key,
                )
                if source_edge_id is not None:
                    (
                        key,
                        source_feature,
                        source_feature_id,
                    ) = _source_edge_station(
                        source_edge_id,
                        position,
                        edges=edges,
                        positions=positions,
                    )
                else:
                    key = (
                        "rail-in-plane-cut",
                        component_id,
                        constraint_key,
                        tuple(
                            sorted((vertex_a.key, vertex_b.key), key=repr)
                        ),
                    )
                    source_feature = "IN_PLANE_CUT"
                    source_feature_id = constraint_key
                station_index = None
                route_ids = tuple(
                    sorted(set(vertex_a.route_ids).union(vertex_b.route_ids))
                )
            output.append(
                RailDomainVertex(
                    key=key,
                    position=position,
                    s=s,
                    r=r,
                    boundary_path_id=path_id,
                    source_feature=source_feature,
                    source_feature_id=source_feature_id,
                    source_edge_id=source_edge_id,
                    source_face_id=current.source_face_id,
                    route_ids=route_ids,
                    station_index=station_index,
                )
            )
        if current_inside:
            output.append(current)
    deduplicated = []
    for vertex in output:
        coordinate = vertex.s if axis == "s" else vertex.r
        arithmetic = 1e-12 * max(1.0, abs(coordinate), abs(bound))
        if path_hint is not None and abs(coordinate - bound) <= arithmetic:
            route_ids = set(vertex.route_ids)
            if path_hint[:1] == ("ROUTE",):
                route_ids.add(path_hint[1])
            vertex = replace(
                vertex,
                boundary_path_id=path_hint,
                route_ids=tuple(sorted(route_ids)),
            )
        if deduplicated and deduplicated[-1].key == vertex.key:
            continue
        deduplicated.append(vertex)
    if deduplicated and deduplicated[0].key == deduplicated[-1].key:
        deduplicated.pop()
    return tuple(deduplicated)


def _triangulate_in_plane_source_polygon(polygon, normal):
    """RV2 для concave source-face: atoms без rail-семантики внутренних рёбер."""

    points = _project_polygon(
        tuple(
            item.position if hasattr(item, "position") else item
            for item in polygon
        ),
        normal,
    )
    signed_area = sum(
        point_a[0] * point_b[1] - point_b[0] * point_a[1]
        for point_a, point_b in zip(points, points[1:] + points[:1])
    )
    if signed_area == 0.0:
        return ()
    order = list(range(len(polygon)))
    if signed_area < 0.0:
        order.reverse()
    triangles = []
    guard = len(order) * len(order)
    while len(order) > 3 and guard > 0:
        ear_found = False
        for position, current in enumerate(order):
            previous = order[position - 1]
            following = order[(position + 1) % len(order)]
            turn = _orientation2(
                points[previous],
                points[current],
                points[following],
            )
            tolerance = _polygon_arithmetic_tolerance(
                (points[previous], points[current], points[following]),
            )
            if turn <= tolerance:
                continue
            triangle_points = (
                points[previous],
                points[current],
                points[following],
            )
            if any(
                candidate not in {previous, current, following}
                and _point_in_polygon2(
                    points[candidate],
                    triangle_points,
                    tolerance,
                )
                and not any(
                    _point_on_segment2(
                        points[candidate],
                        triangle_points[index],
                        triangle_points[(index + 1) % 3],
                        tolerance,
                    )
                    for index in range(3)
                )
                for candidate in order
            ):
                continue
            triangles.append(tuple(polygon[index] for index in (previous, current, following)))
            del order[position]
            ear_found = True
            break
        if not ear_found:
            return ()
        guard -= 1
    if len(order) == 3:
        triangles.append(tuple(polygon[index] for index in order))
    return tuple(triangles)


def _compile_in_plane_channel_cells(
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
    """RP1: source-face intersection с аналитическим planar ribbon."""

    faces = _face_by_id(rail_plan)
    positions = _position_by_vertex(rail_plan)
    face = faces[initial_face_id]
    point_a = positions[component.vertex_ids[interval.from_station_index]]
    point_b = positions[component.vertex_ids[interval.to_station_index]]
    tangent = _normalized3(_sub3(point_b, point_a))
    if tangent is None:
        raise _RailGeometryCompileError(
            "RAIL_GEOMETRY_SPINE_EDGE_DEGENERATE",
            edge_indices=(interval.source_edge_id,),
        )
    side_sign = _side_sign(
        face,
        interval.source_edge_id,
        component.vertex_ids[interval.from_station_index],
        component.vertex_ids[interval.to_station_index],
    )
    lateral = _normalized3(_cross3(face.normal, tangent))
    if lateral is None:
        raise _RailGeometryCompileError(
            "RAIL_GEOMETRY_IN_PLANE_LATERAL_INVALID",
            edge_indices=(interval.source_edge_id,),
            face_indices=(initial_face_id,),
    )
    lateral = _mul3(lateral, -side_sign)
    s_a = component.stations[interval.from_station_index].s
    s_b = component.stations[interval.to_station_index].s
    maximum_window = (
        point_a,
        point_b,
        _add3(point_b, _mul3(lateral, rail_plan.alpha_budget)),
        _add3(point_a, _mul3(lateral, rail_plan.alpha_budget)),
    )
    station_lookup = _in_plane_station_lookup(path_a, path_b)
    region_face_ids = _planar_region_face_ids(rail_plan, initial_face_id)
    edges = _edge_by_id(rail_plan)
    edge_id_by_vertices = {
        frozenset(edge.vertex_ids): edge.edge_id
        for edge in rail_plan.edges
    }

    def fold_intersects_window(edge):
        endpoint_values = []
        for vertex_id in edge.vertex_ids:
            relative = _sub3(positions[vertex_id], point_a)
            endpoint_values.append(
                (
                    s_a + _dot3(relative, tangent),
                    _dot3(relative, lateral),
                )
            )
        lower = 0.0
        upper = 1.0
        for coordinate_index, minimum, maximum in (
            (0, s_a, s_b),
            (1, 0.0, rail_plan.alpha_budget),
        ):
            value_a = endpoint_values[0][coordinate_index]
            value_b = endpoint_values[1][coordinate_index]
            delta = value_b - value_a
            if delta == 0.0:
                if value_a < minimum or value_a > maximum:
                    return False
                continue
            t_min = (minimum - value_a) / delta
            t_max = (maximum - value_a) / delta
            if t_min > t_max:
                t_min, t_max = t_max, t_min
            lower = max(lower, t_min)
            upper = min(upper, t_max)
            if upper < lower:
                return False
        return upper >= lower

    blocking_folds = tuple(
        sorted(
            {
                edge_id
                for face_id in region_face_ids
                for edge_id in faces[face_id].edge_ids
                if edges[edge_id].is_fold
                and not edges[edge_id].is_pchain
                and not edges[edge_id].is_spine
                and fold_intersects_window(edges[edge_id])
            }
        )
    )
    if blocking_folds:
        raise _RailGeometryCompileError(
            "RAIL_GEOMETRY_CURVED_PENDING_R3",
            edge_indices=blocking_folds,
            face_indices=region_face_ids,
        )
    cells = []
    constraints = (
        ("r", 0.0, True, None, (interval.source_edge_id, "SPINE")),
        (
            "r",
            float(rail_plan.alpha_budget),
            False,
            None,
            (interval.source_edge_id, "ALPHA"),
        ),
        ("s", s_a, True, path_a.path_id, (path_a.path_id, "FROM")),
        ("s", s_b, False, path_b.path_id, (path_b.path_id, "TO")),
    )
    for face_id in region_face_ids:
        owner_face = faces[face_id]
        source_polygon = tuple(
            _in_plane_domain_vertex(
                position=positions[vertex_id],
                source_vertex_id=vertex_id,
                face_id=face_id,
                point_a=point_a,
                tangent=tangent,
                lateral=lateral,
                s_a=s_a,
                component_id=component.component_id,
                station_lookup=station_lookup,
            )
            for vertex_id in owner_face.vertex_ids
        )
        if _source_face_is_convex(owner_face, positions):
            source_atoms = (source_polygon,)
        else:
            source_points = tuple(
                vertex.position for vertex in source_polygon
            )
            if not _coplanar_polygons_overlap(
                source_points,
                maximum_window,
                face.normal,
            ):
                continue
            source_atoms = _triangulate_in_plane_source_polygon(
                source_polygon,
                owner_face.normal,
            )
            if not source_atoms:
                raise _RailGeometryCompileError(
                    "RAIL_GEOMETRY_SOURCE_FACE_TRIANGULATION_INVALID",
                    edge_indices=owner_face.edge_ids,
                    vertex_indices=owner_face.vertex_ids,
                    face_indices=(face_id,),
                )
        for source_atom in source_atoms:
            polygon = source_atom
            for axis, bound, keep_greater, path_hint, constraint_key in constraints:
                polygon = _clip_in_plane_polygon(
                    polygon,
                    axis=axis,
                    bound=bound,
                    keep_greater=keep_greater,
                    path_hint=path_hint,
                    constraint_key=constraint_key,
                    component_id=component.component_id,
                    station_lookup=station_lookup,
                    edge_id_by_vertices=edge_id_by_vertices,
                    edges=edges,
                    positions=positions,
                )
                if len(polygon) < 3:
                    break
            if len(polygon) < 3:
                continue
            scale = max(
                1.0,
                *(
                    _length3(_sub3(vertex.position, point_a))
                    for vertex in polygon
                ),
            )
            if _polygon_area_measure(polygon) <= scale * scale * 1e-12:
                continue
            polygon_normal = _polygon_normal(polygon)
            if polygon_normal is None:
                continue
            if _dot3(polygon_normal, owner_face.normal) < 0.0:
                polygon = tuple(reversed(polygon))
            cells.append(
                RailChannelCell(
                    cell_id=first_cell_id + len(cells),
                    channel_id=channel_id,
                    owner_face_id=face_id,
                    spine_edge_id=interval.source_edge_id,
                    source_edge_ids=tuple(
                        sorted({interval.source_edge_id, *owner_face.edge_ids})
                    ),
                    boundary_path_ids=(path_a.path_id, path_b.path_id),
                    vertices=polygon,
                )
            )
    if cells:
        vertex_ids_by_position = defaultdict(list)
        for source_vertex in rail_plan.vertices:
            vertex_ids_by_position[source_vertex.position].append(
                source_vertex.vertex_id
            )

        def source_vertex_id(vertex):
            if vertex.key[:1] == ("rail-source-vertex",):
                return vertex.key[1]
            candidates = vertex_ids_by_position.get(vertex.position, ())
            return candidates[0] if len(candidates) == 1 else None

        edge_owners = defaultdict(list)
        for cell_index, cell in enumerate(cells):
            vertices = cell.vertices
            for vertex_a, vertex_b in zip(
                vertices, vertices[1:] + vertices[:1]
            ):
                key_a = vertex_a.key
                key_b = vertex_b.key
                source_edge_id = None
                source_vertex_a = source_vertex_id(vertex_a)
                source_vertex_b = source_vertex_id(vertex_b)
                if source_vertex_a is not None and source_vertex_b is not None:
                    source_edge_id = edge_id_by_vertices.get(
                        frozenset((source_vertex_a, source_vertex_b))
                    )
                is_barrier = source_edge_id is not None and (
                    edges[source_edge_id].is_pchain
                    or edges[source_edge_id].is_spine
                    or edges[source_edge_id].is_fold
                )
                edge_owners[frozenset((key_a, key_b))].append(
                    (cell_index, is_barrier)
                )
        adjacency = {index: set() for index in range(len(cells))}
        for owners in edge_owners.values():
            if len(owners) != 2:
                continue
            if owners[0][1] or owners[1][1]:
                continue
            adjacency[owners[0][0]].add(owners[1][0])
            adjacency[owners[1][0]].add(owners[0][0])
        reachable = {
            index
            for index, cell in enumerate(cells)
            if sum(vertex.r == 0.0 for vertex in cell.vertices) >= 2
        }
        pending = list(reachable)
        while pending:
            current = pending.pop()
            for neighbor in adjacency[current].difference(reachable):
                reachable.add(neighbor)
                pending.append(neighbor)
        cells = [
            replace(cell, cell_id=first_cell_id + kept_index)
            for kept_index, cell in enumerate(
                cell
                for index, cell in enumerate(cells)
                if index in reachable
            )
        ]
    if not cells:
        raise _RailGeometryCompileError(
            "RAIL_GEOMETRY_IN_PLANE_CHANNEL_EMPTY",
            edge_indices=(interval.source_edge_id,),
            face_indices=region_face_ids,
        )
    return tuple(cells)


def _route_for_sector_edges(
    rail_plan,
    sector,
    edge_ids,
    failure_reason,
):
    candidates = tuple(
        route
        for route in rail_plan.routes
        if route.key.side.spine_vertex_id == sector.spine_vertex_id
        and route.key.side.start_edge_id in edge_ids
    )
    if len(candidates) != 1:
        raise _RailGeometryCompileError(
            failure_reason,
            edge_indices=edge_ids,
            vertex_indices=(sector.spine_vertex_id,),
            face_indices=sector.source_face_ids,
            route_ids=(route.route_id for route in candidates),
            details=(
                ("sector_id", sector.sector_id),
                ("candidate_count", len(candidates)),
            ),
        )
    return candidates[0]


def _cached_route_path(rail_plan, route, path_by_id):
    path_id = ("ROUTE", route.route_id)
    cached = path_by_id.get(path_id)
    if cached is not None:
        return cached
    path = _route_boundary_path(rail_plan, route)
    path_by_id[path.path_id] = path
    return path


def _compile_angular_pieces(
    *,
    rail_plan,
    angular_channel_id,
    sector,
    spine_edge_id,
    apex_s,
    initial_face_id,
    ordinary_path,
    boundary_path,
    first_piece_id,
):
    """RM7a per-face fan: оба берега имеют свой r и общий apex-s."""

    faces = _face_by_id(rail_plan)
    current_face_id = initial_face_id
    minimum_ordinary = 0
    minimum_boundary = 0
    visited = set()
    pieces = []
    while current_face_id is not None:
        if current_face_id in visited:
            raise _RailGeometryCompileError(
                "RAIL_GEOMETRY_ANGULAR_FACE_CYCLE",
                face_indices=(current_face_id,),
                route_ids=(ordinary_path.route_id, boundary_path.route_id),
                details=(("sector_id", sector.sector_id),),
            )
        visited.add(current_face_id)
        face = faces[current_face_id]
        if face.planarity_min_dot < _RAIL_PLANAR_DOT:
            raise _RailGeometryCompileError(
                "RAIL_GEOMETRY_SOURCE_FACE_NON_PLANAR",
                face_indices=(current_face_id,),
            )
        ordinary_piece = _piece_for_face(
            ordinary_path,
            current_face_id,
            minimum_ordinary,
        )
        boundary_piece = _piece_for_face(
            boundary_path,
            current_face_id,
            minimum_boundary,
        )
        if ordinary_piece is None or boundary_piece is None:
            if not pieces:
                raise _RailGeometryCompileError(
                    "RAIL_GEOMETRY_ANGULAR_BOUNDARY_MISSING",
                    edge_indices=(spine_edge_id,),
                    vertex_indices=(sector.spine_vertex_id,),
                    face_indices=(current_face_id,),
                    route_ids=(ordinary_path.route_id, boundary_path.route_id),
                    details=(("sector_id", sector.sector_id),),
                )
            break
        minimum_ordinary = ordinary_piece.piece_index + 1
        minimum_boundary = boundary_piece.piece_index + 1
        vertices = (
            _domain_vertex(
                ordinary_piece.start,
                s=apex_s,
                face_id=current_face_id,
                path_id=ordinary_path.path_id,
            ),
            _domain_vertex(
                boundary_piece.start,
                s=apex_s,
                face_id=current_face_id,
                path_id=boundary_path.path_id,
            ),
            _domain_vertex(
                boundary_piece.end,
                s=apex_s,
                face_id=current_face_id,
                path_id=boundary_path.path_id,
            ),
            _domain_vertex(
                ordinary_piece.end,
                s=apex_s,
                face_id=current_face_id,
                path_id=ordinary_path.path_id,
            ),
        )
        polygon_normal = _polygon_normal(vertices)
        if polygon_normal is None:
            raise _RailGeometryCompileError(
                "RAIL_GEOMETRY_ANGULAR_PIECE_DEGENERATE",
                edge_indices=(spine_edge_id,),
                vertex_indices=(sector.spine_vertex_id,),
                face_indices=(current_face_id,),
                route_ids=(ordinary_path.route_id, boundary_path.route_id),
                details=(("sector_id", sector.sector_id),),
            )
        if _dot3(polygon_normal, face.normal) < 0.0:
            vertices = tuple(reversed(vertices))
        pieces.append(
            RailAngularPiece(
                piece_id=first_piece_id + len(pieces),
                angular_channel_id=angular_channel_id,
                owner_face_id=current_face_id,
                spine_edge_id=spine_edge_id,
                sector_id=sector.sector_id,
                source_edge_ids=tuple(
                    sorted(
                        {
                            *ordinary_piece.source_edge_ids,
                            *boundary_piece.source_edge_ids,
                        }
                    )
                ),
                boundary_path_ids=(
                    ordinary_path.path_id,
                    boundary_path.path_id,
                ),
                vertices=vertices,
            )
        )
        current_face_id = _next_face(
            ordinary_path,
            ordinary_piece,
            boundary_path,
            boundary_piece,
            current_face_id,
        )
    return tuple(pieces)


def _compile_angular_channels(rail_plan, component, path_by_id):
    """Компилирует только RR1a route-сектора spine+boundary."""

    edges = _edge_by_id(rail_plan)
    faces = _face_by_id(rail_plan)
    component_edge_ids = set(component.edge_ids)
    interval_by_edge = {
        interval.source_edge_id: interval for interval in component.intervals
    }
    station_by_vertex = {
        station.source_vertex_id: station for station in component.stations
    }
    angular_channels = []
    next_piece_id = 0
    for sector in rail_plan.start_sectors:
        if (
            sector.kind != RailStartSectorKind.ROUTE
            or sector.spine_vertex_id not in station_by_vertex
        ):
            continue
        spine_delimiters = tuple(
            edge_id
            for edge_id in sector.delimiter_edge_ids
            if edge_id in component_edge_ids
        )
        boundary_delimiters = tuple(
            edge_id
            for edge_id in sector.delimiter_edge_ids
            if edge_id not in component_edge_ids and edges[edge_id].is_pchain
        )
        if not boundary_delimiters:
            # spine-spine ROUTE-сектор остаётся обычным RR1-rail.
            continue
        if not spine_delimiters:
            # Boundary-boundary сектор не является RM7a-интерфейсом spine.
            continue
        if len(spine_delimiters) != 1 or len(boundary_delimiters) != 1:
            raise _RailGeometryCompileError(
                "RAIL_GEOMETRY_ANGULAR_DELIMITER_AMBIGUOUS",
                edge_indices=sector.delimiter_edge_ids,
                vertex_indices=(sector.spine_vertex_id,),
                face_indices=sector.source_face_ids,
                details=(("sector_id", sector.sector_id),),
            )
        spine_edge_id = spine_delimiters[0]
        boundary_edge_id = boundary_delimiters[0]
        ordinary_route = _route_for_sector_edges(
            rail_plan,
            sector,
            set(sector.internal_edge_ids),
            "RAIL_GEOMETRY_ANGULAR_ORDINARY_ROUTE_AMBIGUOUS",
        )
        boundary_route = _route_for_sector_edges(
            rail_plan,
            sector,
            {boundary_edge_id},
            "RAIL_GEOMETRY_ANGULAR_BOUNDARY_ROUTE_AMBIGUOUS",
        )
        ordinary_path = _cached_route_path(
            rail_plan,
            ordinary_route,
            path_by_id,
        )
        boundary_path = _cached_route_path(
            rail_plan,
            boundary_route,
            path_by_id,
        )
        ordinary_faces = {piece.owner_face_id for piece in ordinary_path.pieces}
        boundary_faces = {piece.owner_face_id for piece in boundary_path.pieces}
        initial_faces = tuple(
            sorted(
                set(sector.source_face_ids)
                .intersection(ordinary_faces)
                .intersection(boundary_faces)
            )
        )
        if len(initial_faces) != 1:
            raise _RailGeometryCompileError(
                "RAIL_GEOMETRY_ANGULAR_OWNER_AMBIGUOUS",
                edge_indices=(spine_edge_id, boundary_edge_id),
                vertex_indices=(sector.spine_vertex_id,),
                face_indices=initial_faces or sector.source_face_ids,
                route_ids=(ordinary_route.route_id, boundary_route.route_id),
                details=(
                    ("sector_id", sector.sector_id),
                    ("candidate_count", len(initial_faces)),
                ),
            )
        spine_owner_faces = tuple(
            sorted(
                set(sector.source_face_ids).intersection(
                    edges[spine_edge_id].face_indices
                )
            )
        )
        if len(spine_owner_faces) != 1:
            raise _RailGeometryCompileError(
                "RAIL_GEOMETRY_ANGULAR_SIDE_AMBIGUOUS",
                edge_indices=(spine_edge_id,),
                vertex_indices=(sector.spine_vertex_id,),
                face_indices=spine_owner_faces or sector.source_face_ids,
                details=(("sector_id", sector.sector_id),),
            )
        interval = interval_by_edge[spine_edge_id]
        side_sign = _side_sign(
            faces[spine_owner_faces[0]],
            spine_edge_id,
            component.vertex_ids[interval.from_station_index],
            component.vertex_ids[interval.to_station_index],
        )
        apex_s = station_by_vertex[sector.spine_vertex_id].s
        pieces = _compile_angular_pieces(
            rail_plan=rail_plan,
            angular_channel_id=len(angular_channels),
            sector=sector,
            spine_edge_id=spine_edge_id,
            apex_s=apex_s,
            initial_face_id=initial_faces[0],
            ordinary_path=ordinary_path,
            boundary_path=boundary_path,
            first_piece_id=next_piece_id,
        )
        next_piece_id += len(pieces)
        angular_channels.append(
            RailAngularChannel(
                angular_channel_id=len(angular_channels),
                sector_id=sector.sector_id,
                spine_vertex_id=sector.spine_vertex_id,
                spine_edge_id=spine_edge_id,
                side_sign=side_sign,
                apex_s=apex_s,
                ordinary_path_id=ordinary_path.path_id,
                boundary_path_id=boundary_path.path_id,
                pieces=pieces,
            )
        )
    return tuple(angular_channels)


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
    angular_channels,
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
    angular_by_vertex_sign = defaultdict(list)
    for angular_channel in angular_channels:
        angular_by_vertex_sign[
            (angular_channel.spine_vertex_id, angular_channel.side_sign)
        ].append(angular_channel)
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
            angular_candidates = tuple(
                angular_by_vertex_sign.get(
                    (corner_vertex_id, side_sign),
                    (),
                )
            )
            if angular_candidates:
                expected_spine_edges = {
                    previous_interval.source_edge_id,
                    next_interval.source_edge_id,
                }
                actual_spine_edges = {
                    channel.spine_edge_id for channel in angular_candidates
                }
                boundary_path_ids = {
                    channel.boundary_path_id for channel in angular_candidates
                }
                complete_coverage = (
                    len(angular_candidates) == 2
                    and actual_spine_edges == expected_spine_edges
                    and len(boundary_path_ids) == 1
                )
                if not complete_coverage:
                    raise _RailGeometryCompileError(
                        "RAIL_GEOMETRY_ANGULAR_COVERAGE_PARTIAL",
                        edge_indices=expected_spine_edges.union(
                            actual_spine_edges
                        ),
                        vertex_indices=(corner_vertex_id,),
                        face_indices=tuple(
                            sorted(
                                {
                                    piece.owner_face_id
                                    for channel in angular_candidates
                                    for piece in channel.pieces
                                }
                            )
                        ),
                        route_ids=(
                            path_by_id[channel.ordinary_path_id].route_id
                            for channel in angular_candidates
                        ),
                        details=(
                            ("side_sign", side_sign),
                            (
                                "angular_channel_ids",
                                tuple(
                                    channel.angular_channel_id
                                    for channel in angular_candidates
                                ),
                            ),
                        ),
                    )
                # RM7a полностью закрыл vertex-side; RM6 здесь запрещён.
                continue
            shared_boundary_clip = (
                previous_path.kind == "ROUTE"
                and next_path.kind == "ROUTE"
                and previous_path.path_id == next_path.path_id
                and _edge_by_id(rail_plan)[
                    next(
                        route.key.side.start_edge_id
                        for route in rail_plan.routes
                        if route.route_id == previous_path.route_id
                    )
                ].is_pchain
            )
            if _dot3(previous_face.normal, next_face.normal) < _RAIL_PLANAR_DOT:
                if shared_boundary_clip:
                    # RP3/RR8a: две planar-стороны заканчиваются на одной
                    # pChain clip-line; cross-fold A10 здесь не существует.
                    continue
                raise _RailGeometryCompileError(
                    "RAIL_GEOMETRY_CORNER_MECHANISM_MISSING",
                    face_indices=(previous_face.face_id, next_face.face_id),
                    vertex_indices=(corner_vertex_id,),
                    details=(("side_sign", side_sign),),
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
            corner_pair = (
                previous_path.kind == "CORNER"
                and next_path.kind == "CORNER"
                and previous_path.corner_sector_id
                == next_path.corner_sector_id
                and previous_path.corner_side == side_sign
                and next_path.corner_side == side_sign
            )
            shared_route = (
                previous_path.kind == "ROUTE"
                and next_path.kind == "ROUTE"
                and previous_path.path_id == next_path.path_id
            )
            if cap_pair or corner_pair:
                lateral_prev = _path_initial_direction(
                    previous_path,
                    corner_vertex_id,
                )
                lateral_next = _path_initial_direction(
                    next_path,
                    corner_vertex_id,
                )
                direction_failure = (
                    "RAIL_GEOMETRY_CORNER_PATH_DIRECTION_INVALID"
                    if corner_pair
                    else "RAIL_GEOMETRY_CORNER_CAP_DIRECTION_INVALID"
                )
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
            owner_face_ids = ()
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
            elif corner_pair:
                sector = next(
                    (
                        sector
                        for sector in rail_plan.start_sectors
                        if sector.sector_id == previous_path.corner_sector_id
                    ),
                    None,
                )
                owner_face_ids = (
                    previous_path.owner_face_ids
                    if previous_path.owner_face_ids
                    else sector.source_face_ids
                )
                non_planar = tuple(
                    face_id
                    for face_id in owner_face_ids
                    if _dot3(faces[face_id].normal, previous_face.normal)
                    < _RAIL_PLANAR_DOT
                )
                if non_planar:
                    raise _RailGeometryCompileError(
                        "RAIL_GEOMETRY_CORNER_SECTOR_NON_PLANAR",
                        edge_indices=(
                            sector.delimiter_edge_ids
                            if sector is not None
                            else (
                                previous_interval.source_edge_id,
                                next_interval.source_edge_id,
                            )
                        ),
                        vertex_indices=(corner_vertex_id,),
                        face_indices=non_planar,
                        details=(
                            ("sector_id", previous_path.corner_sector_id),
                        ),
                    )
                if (
                    previous_channel.initial_face_id
                    == next_channel.initial_face_id
                ):
                    mode = "INNER_DIVIDER"
                    owner_face_id = previous_channel.initial_face_id
                else:
                    mode = "OUTER_FILL"
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
                    owner_face_ids=tuple(owner_face_ids),
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
    partition_keys = {
        (partition.source_vertex_id, partition.side_sign)
        for partition in partitions
    }
    angular_keys = {
        (channel.spine_vertex_id, channel.side_sign)
        for channel in angular_channels
    }
    conflicts = tuple(sorted(partition_keys.intersection(angular_keys)))
    if conflicts:
        raise _RailGeometryCompileError(
            "RAIL_GEOMETRY_CORNER_MECHANISM_CONFLICT",
            vertex_indices=(vertex_id for vertex_id, _side in conflicts),
            details=(("vertex_sides", conflicts),),
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
                            / path_scales.get(vertex.boundary_path_id, 1.0)
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


def _normalize_angular_channels(angular_channels, path_by_id, path_scales):
    normalized = []
    for channel in angular_channels:
        path_ids = (channel.ordinary_path_id, channel.boundary_path_id)
        alpha_limit = min(
            _path_reach(path_by_id[path_id]) / path_scales[path_id]
            for path_id in path_ids
        )
        pieces = tuple(
            replace(
                piece,
                vertices=tuple(
                    replace(
                        vertex,
                        r=vertex.r
                        / path_scales.get(vertex.boundary_path_id, 1.0),
                    )
                    for vertex in piece.vertices
                ),
            )
            for piece in channel.pieces
        )
        normalized.append(
            replace(channel, pieces=pieces, alpha_limit=alpha_limit)
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
    component_id,
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
                route_id
                for route_id in (
                    path_by_id[partition.previous_path_id].route_id,
                    path_by_id[partition.next_path_id].route_id,
                )
                if route_id is not None
            }
        )
    )
    radius = _corner_radius(partition, position, normal)
    arithmetic = 1e-12 * max(1.0, abs(radius))
    if abs(radius) <= arithmetic:
        radius = 0.0
    if abs(radius - partition.alpha_limit) <= arithmetic:
        radius = partition.alpha_limit
    canonical_miter = _add3(
        partition.point,
        _mul3(partition.miter_vector, partition.alpha_limit),
    )
    if _length3(_sub3(position, canonical_miter)) <= arithmetic:
        position = canonical_miter
        radius = partition.alpha_limit
        key = (
            "rail-corner-miter",
            component_id,
            partition.partition_id,
            radius,
        )
        path_id = ("CORNER", partition.partition_id)
        source_feature = "CORNER_MITER"
        source_feature_id = (partition.partition_id, radius)
        source_edge_id = None
        station_index = None
    if (
        path_hint is not None
        and path_hint[:1]
        in {
            ("CORNER_PATH",),
            ("IN_PLANE_PATH",),
            ("IN_PLANE_JOIN_PATH",),
        }
        and key[:1] == ("rail-corner-compile-cut",)
    ):
        key = ("rail-frontier", component_id, path_hint, radius)
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
    component_id,
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
                    component_id=component_id,
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
        if partition.mode != "OUTER_FILL":
            continue
        if partition.owner_face_ids:
            topology_reachable = frozenset(partition.owner_face_ids)
            owner_face = faces[min(topology_reachable)]
        elif partition.owner_face_id is not None:
            owner_face = faces[partition.owner_face_id]
            topology_reachable = _corner_topology_reachable_faces(
                partition.owner_face_id,
                owner_face.normal,
                faces,
                edges,
                rail_edge_ids,
                dam_vertex_ids,
            )
        else:
            continue
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
        for face_id in sorted(topology_reachable):
            face = faces[face_id]
            if _dot3(face.normal, normal) < _RAIL_PLANAR_DOT:
                continue
            polygon = tuple(
                _corner_compile_vertex(
                    component_id=component_id,
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
            if _source_face_is_convex(face, vertices):
                source_atoms = (polygon,)
            else:
                source_points = tuple(vertex.position for vertex in polygon)
                if not _coplanar_polygons_overlap(
                    source_points,
                    maximum_window,
                    normal,
                ):
                    continue
                source_atoms = _triangulate_in_plane_source_polygon(
                    polygon,
                    face.normal,
                )
                if not source_atoms:
                    raise _RailGeometryCompileError(
                        "RAIL_GEOMETRY_SOURCE_FACE_TRIANGULATION_INVALID",
                        edge_indices=face.edge_ids,
                        vertex_indices=face.vertex_ids,
                        face_indices=(face_id,),
                        details=(
                            ("corner_vertex_id", partition.source_vertex_id),
                            ("corner_partition_id", partition.partition_id),
                        ),
                    )
            face_polygons = []
            for source_atom in source_atoms:
                polygon = source_atom
                for line_start, line_end, line_key, path_hint in clip_lines:
                    polygon = _clip_corner_polygon_by_line(
                        polygon,
                        component_id=component_id,
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
                face_polygons.extend(
                    split_polygon
                    for split_polygon in split_polygons
                    if len(split_polygon) >= 3
                    and _polygon_area_measure(split_polygon)
                    > scale * scale * 1e-12
                )
            if face_polygons:
                candidate_polygons[face_id] = tuple(face_polygons)

        if partition.owner_face_ids:
            # RM6a owner уже является полным sector-set; seed/flood выбора нет.
            reachable = set(candidate_polygons)
        elif partition.owner_face_id not in candidate_polygons:
            raise _RailGeometryCompileError(
                "RAIL_GEOMETRY_CORNER_OWNER_CELL_MISSING",
                vertex_indices=(partition.source_vertex_id,),
                face_indices=(partition.owner_face_id,),
            )
        else:
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
                        if (
                            neighbor_id in candidate_ids
                            and neighbor_id not in reachable
                        ):
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
        for face_id in interval.source_face_ids:
            if faces[face_id].planarity_min_dot < _RAIL_PLANAR_DOT:
                raise _RailGeometryCompileError(
                    "RAIL_GEOMETRY_SOURCE_FACE_NON_PLANAR",
                    edge_indices=(interval.source_edge_id,),
                    face_indices=(face_id,),
                )
    for interval in component.intervals:
        spine_edge = _edge_by_id(rail_plan)[interval.source_edge_id]
        for face_id in interval.source_face_ids:
            face = faces[face_id]
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
            route_paths = tuple(
                path for path in (path_a, path_b) if path.kind == "ROUTE"
            )
            compiler = (
                _compile_channel_cells
                if route_paths
                and all(
                    any(
                        piece.owner_face_id == face_id
                        for piece in path.pieces
                    )
                    for path in route_paths
                )
                else _compile_in_plane_channel_cells
            )
            # RP3/RR8a: pChain — явная clip-линия. Её станционные позиции,
            # а не ортогональный in-plane суррогат, владеют границей русла
            # и остаются общими для всех смежных секторов.
            cells = compiler(
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
    # RP3: RailAngularChannel остаётся geometry-IR примитивом R3, но R1.1
    # намеренно не активирует sector/angular materialization.
    angular_channels = ()
    corner_partitions = _compile_corner_partitions(
        rail_plan,
        component,
        channels,
        angular_channels,
        path_by_id,
        apex_limit,
        split_angle,
    )
    path_scales = _compile_path_reach_scales(
        path_by_id,
        corner_partitions,
    )
    channels = _normalize_channels(channels, path_by_id, path_scales)
    angular_channels = _normalize_angular_channels(
        angular_channels,
        path_by_id,
        path_scales,
    )
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
        angular_channels=tuple(angular_channels),
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
    join_mode="MITER",
):
    """Compile-only PLANAR admission одного физического spine-компонента."""

    try:
        join_mode_value = str(
            getattr(join_mode, "value", join_mode)
        ).upper()
        if join_mode_value not in {"MITER", "BEVEL"}:
            raise _RailGeometryCompileError(
                "RAIL_GEOMETRY_JOIN_MODE_UNSUPPORTED",
                edge_indices=edge_indices,
            )
        if join_mode_value == "BEVEL":
            # R1 rail-corner cells пока поддерживают только stable A10
            # MITER. Явный отказ маршрутизирует selection в Patch backend,
            # где переключатель не будет тихо проигнорирован.
            raise _RailGeometryCompileError(
                "RAIL_GEOMETRY_BEVEL_UNSUPPORTED",
                edge_indices=edge_indices,
            )
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


def _interpolate_domain_vertex(
    vertex_a,
    vertex_b,
    alpha,
    component_id,
    *,
    edge_id_by_vertices,
    edges,
    positions,
):
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
    source_edge_id = _source_edge_for_domain_segment(
        vertex_a,
        vertex_b,
        edge_id_by_vertices=edge_id_by_vertices,
        edges=edges,
    )
    if source_edge_id is not None and shared_path_id is None:
        key, source_feature, source_feature_id = _source_edge_station(
            source_edge_id,
            position,
            edges=edges,
            positions=positions,
        )
        path_id = shared_path_id or ("SOURCE_EDGE", source_edge_id)
    elif shared_path_id is not None:
        analytic_specs = tuple(
            vertex.source_feature_id
            for vertex in (vertex_a, vertex_b)
            if isinstance(vertex.source_feature_id, tuple)
            and vertex.source_feature_id[:1] == ("ANALYTIC_PATH",)
            and vertex.source_feature_id[1] == shared_path_id
        )
        if analytic_specs:
            origins = {spec[2] for spec in analytic_specs}
            directions = {spec[3] for spec in analytic_specs}
            if len(origins) == 1 and len(directions) == 1:
                position = _add3(
                    next(iter(origins)),
                    _mul3(next(iter(directions)), alpha),
                )
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
        source_feature = "RAIL_FRONTIER"
        source_feature_id = (vertex_a.key, vertex_b.key, alpha)
    else:
        ordered_keys = tuple(sorted((vertex_a.key, vertex_b.key), key=repr))
        key = ("rail-frontier-cell", component_id, ordered_keys, alpha)
        path_id = ("CELL", ordered_keys)
        source_feature = "RAIL_FRONTIER"
        source_feature_id = (vertex_a.key, vertex_b.key, alpha)
    return RailDomainVertex(
        key=key,
        position=position,
        s=s,
        r=alpha,
        boundary_path_id=path_id,
        source_feature=source_feature,
        source_feature_id=source_feature_id,
        source_edge_id=source_edge_id,
        source_face_id=vertex_a.source_face_id,
        route_ids=tuple(sorted(set(vertex_a.route_ids).union(vertex_b.route_ids))),
        station_index=None,
    )


def _clip_cell(
    cell,
    alpha,
    component_id,
    *,
    edge_id_by_vertices,
    edges,
    positions,
):
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
                    edge_id_by_vertices=edge_id_by_vertices,
                    edges=edges,
                    positions=positions,
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
                (
                    "element_id",
                    getattr(cell, "cell_id", getattr(cell, "piece_id", None)),
                ),
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
    arithmetic = 1e-12 * max(1.0, abs(r), abs(vertex_a.r), abs(vertex_b.r))
    if abs(r) <= arithmetic:
        return replace(
            vertex_a,
            key=("rail-source-vertex", partition.source_vertex_id),
            position=partition.point,
            s=partition.s,
            r=0.0,
            boundary_path_id=("CORNER", partition.partition_id),
            source_feature="SOURCE_VERTEX",
            source_feature_id=partition.source_vertex_id,
            source_edge_id=None,
            station_index=0,
        )
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
    if path.kind == "CORNER":
        by_key = {}
        for piece in path.pieces:
            for vertex in (piece.start, piece.end):
                existing = by_key.get(vertex.key)
                if existing is not None and existing.position != vertex.position:
                    raise RailGeometryEvaluationError(
                        RailGeometryFailure(
                            reason="RAIL_GEOMETRY_BOUNDARY_PATH_DESYNC",
                            details=(("path_id", path.path_id),),
                        )
                    )
                by_key[vertex.key] = vertex
        return tuple(
            sorted(by_key.values(), key=lambda vertex: (vertex.r, repr(vertex.key)))
        )
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
                    route_ids=(
                        () if path.route_id is None else (path.route_id,)
                    ),
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
                    route_ids=(
                        () if path.route_id is None else (path.route_id,)
                    ),
                    details=(("piece_index", piece_index),),
                )
            )
        ordered.append(end)
    return tuple(ordered)


def _apply_canonical_offset_lift(faces, offset):
    """Один raw point и один offset-lift на каждый semantic vertex key."""

    raw_position_by_key = {}
    normal_keys_by_key = defaultdict(set)
    face_ids_by_key = defaultdict(set)
    occurrences = []
    for face in faces:
        normal_key = tuple(float(value) for value in face.surface_normal)
        for vertex_index, (key, position) in enumerate(
            zip(face.vert_keys, face.positions)
        ):
            raw_position = tuple(float(value) for value in position)
            existing = raw_position_by_key.get(key)
            if existing is not None and existing != raw_position:
                raise RailGeometryEvaluationError(
                    RailGeometryFailure(
                        reason="RAIL_GEOMETRY_SHARED_KEY_POSITION_DESYNC",
                        face_indices=tuple(
                            sorted(
                                face_ids_by_key[key].union(
                                    (face.surface_id,)
                                )
                            )
                        ),
                        details=(
                            ("vertex_key", key),
                            ("canonical_raw_position", existing),
                            ("conflicting_raw_position", raw_position),
                        ),
                    )
                )
            raw_position_by_key.setdefault(key, raw_position)
            normal_keys_by_key[key].add(normal_key)
            face_ids_by_key[key].add(face.surface_id)
            occurrences.append((face, vertex_index, key))

    lifted_by_key = {
        key: lift_offset_position(
            Vector(raw_position),
            [Vector(normal) for normal in sorted(normal_keys_by_key[key])],
            float(offset),
            coplanar_dot=_RAIL_PLANAR_DOT,
        )
        for key, raw_position in raw_position_by_key.items()
    }
    for face, vertex_index, key in occurrences:
        face.positions[vertex_index] = lifted_by_key[key].copy()


def _canonicalize_analytic_frontiers(faces, plan):
    """Один analytic raw point на path+alpha независимо от face clipping."""

    path_by_id = {path.path_id: path for path in plan.boundary_paths}
    frames = {}
    for path_id, path in path_by_id.items():
        if path_id[:1] not in {
            ("CORNER_PATH",),
            ("IN_PLANE_PATH",),
            ("IN_PLANE_JOIN_PATH",),
        }:
            continue
        origins = {
            vertex.position
            for piece in path.pieces
            for vertex in (piece.start, piece.end)
            if vertex.r == 0.0
        }
        direction = _path_initial_direction(path, path.spine_vertex_id)
        if len(origins) == 1 and direction is not None:
            frames[path_id] = (next(iter(origins)), direction)
    frontier_r_by_path = defaultdict(set)
    for face in faces:
        for key in face.vert_keys:
            if key[:1] == ("rail-frontier",) and len(key) == 4:
                frontier_r_by_path[key[2]].add(float(key[3]))
    for face in faces:
        for vertex_index, key in enumerate(face.vert_keys):
            if key[:1] == ("rail-frontier",) and len(key) == 4:
                path_id = key[2]
                frame = frames.get(path_id)
                if frame is None:
                    continue
                origin, direction = frame
                face.positions[vertex_index] = Vector(
                    _add3(origin, _mul3(direction, float(key[3])))
                )
                continue
            if key[:1] == ("rail-source-vertex",):
                continue
            provenance = face.rail_provenance
            candidate_paths = (
                ()
                if provenance is None
                else tuple(
                    path_id
                    for path_id in provenance.boundary_path_ids
                    if path_id in frames
                )
            )
            position = tuple(float(value) for value in face.positions[vertex_index])
            matches = []
            for path_id in candidate_paths:
                origin, direction = frames[path_id]
                relative = _sub3(position, origin)
                radius = _dot3(relative, direction)
                projected = _add3(origin, _mul3(direction, radius))
                arithmetic = 1e-12 * max(
                    1.0,
                    abs(radius),
                    _length3(relative),
                )
                if _length3(_sub3(position, projected)) > arithmetic:
                    continue
                if abs(radius) <= arithmetic:
                    matches.append((path_id, 0.0, origin))
                    continue
                if radius < 0.0:
                    continue
                canonical_radii = tuple(
                    value
                    for value in frontier_r_by_path.get(path_id, ())
                    if abs(value - radius) <= arithmetic
                )
                if len(canonical_radii) == 1:
                    radius = canonical_radii[0]
                    projected = _add3(origin, _mul3(direction, radius))
                matches.append((path_id, radius, projected))
            if len(matches) != 1:
                continue
            path_id, radius, projected = matches[0]
            if radius == 0.0:
                face.vert_keys[vertex_index] = (
                    "rail-source-vertex",
                    path_by_id[path_id].spine_vertex_id,
                )
            else:
                face.vert_keys[vertex_index] = (
                    "rail-frontier",
                    plan.component.component_id,
                    path_id,
                    radius,
                )
            face.positions[vertex_index] = Vector(projected)


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
    edges_by_id = _edge_by_id(plan.rail_plan)
    positions_by_vertex = _position_by_vertex(plan.rail_plan)
    edge_id_by_vertices = {
        frozenset(edge.vertex_ids): edge.edge_id
        for edge in plan.rail_plan.edges
    }
    pending = []
    corner_channel_ids = {
        channel_id
        for partition in plan.corner_partitions
        for channel_id in (
            partition.previous_channel_id,
            partition.next_channel_id,
        )
    }
    for channel in plan.channels:
        # RR8b: ``alpha_limit`` — общий reach, до которого оба берега ещё
        # имеют незавершённый route. Он не является пределом самого русла:
        # после конца короткого route соответствующая вершина остаётся на
        # структурной границе, а второй берег продолжает скользить по своим
        # станциям. Сам polygon cell уже хранит оба независимых reach. У
        # настоящего spine-corner общий предел пока принадлежит отдельной
        # corner-partition и сохраняется до её независимой R4-параметризации.
        channel_alpha = (
            min(alpha, channel.alpha_limit)
            if channel.channel_id in corner_channel_ids
            else alpha
        )
        for cell in channel.cells:
            clipped = _clip_cell(
                cell,
                channel_alpha,
                plan.component.component_id,
                edge_id_by_vertices=edge_id_by_vertices,
                edges=edges_by_id,
                positions=positions_by_vertex,
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
            ]
            anchor_records = [
                record
                for record in records
                if record[1].owner_face_id == partition.owner_face_id
            ]
            if len(anchor_records) != 1:
                raise RailGeometryEvaluationError(
                    RailGeometryFailure(
                        reason="RAIL_GEOMETRY_CORNER_CELL_MISSING",
                        vertex_indices=(partition.source_vertex_id,),
                        face_indices=(partition.owner_face_id,),
                        details=(("channel_id", channel_id),),
                    )
                )
            anchor_record = anchor_records[0]
            spine_vertices = [
                vertex
                for vertex in anchor_record[2]
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
            keep_sign = 1 if owner_value > 0.0 else -1
            for record in tuple(records):
                record_face = faces_by_id[record[1].owner_face_id]
                divided = _clip_by_corner_divider(
                    record[2],
                    partition,
                    record_face.normal,
                    keep_sign,
                    plan.component.component_id,
                )
                if not _positive_clipped_extent(
                    record[1], divided, record[3]
                ):
                    pending.remove(record)
                    continue
                record[2] = divided

    output = []
    for channel, cell, clipped, _channel_alpha in pending:
        owner_face = faces_by_id[cell.owner_face_id]
        normal = Vector(owner_face.normal)
        # Временный adapter: R4 заменит эту запись единственным
        # каноническим отображением (s, r) -> (U, V).
        u_fracs = [
            channel.side_sign * vertex.r / alpha for vertex in clipped
        ]
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
                positions=[Vector(vertex.position) for vertex in clipped],
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

    for angular_channel in plan.angular_channels:
        angular_alpha = min(alpha, angular_channel.alpha_limit)
        for piece in angular_channel.pieces:
            vertices = _clip_cell(
                piece,
                angular_alpha,
                plan.component.component_id,
                edge_id_by_vertices=edge_id_by_vertices,
                edges=edges_by_id,
                positions=positions_by_vertex,
            )
            if not _positive_clipped_extent(
                piece,
                vertices,
                angular_alpha,
            ):
                continue
            owner_face = faces_by_id[piece.owner_face_id]
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
                cell_id=None,
                corner_partition_id=None,
                source_face_id=piece.owner_face_id,
                source_edge_ids=piece.source_edge_ids,
                boundary_path_ids=piece.boundary_path_ids,
                route_ids=route_ids,
                station_keys=tuple(vertex.key for vertex in vertices),
                angular_channel_id=angular_channel.angular_channel_id,
                sector_id=angular_channel.sector_id,
            )
            normal = Vector(owner_face.normal)
            output.append(
                RailDecalGeometryFace(
                    surface_id=piece.owner_face_id,
                    surface_normal=normal,
                    vert_keys=[vertex.key for vertex in vertices],
                    positions=[Vector(vertex.position) for vertex in vertices],
                    u_fracs=[
                        angular_channel.side_sign * vertex.r / alpha
                        for vertex in vertices
                    ],
                    v_lengths=[vertex.s for vertex in vertices],
                    component_kind="RAIL_ANGULAR",
                    component_side=(
                        f"component:{plan.component.component_id}:"
                        f"angular:{angular_channel.angular_channel_id}:"
                        f"face:{piece.owner_face_id}"
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
            edge_id_by_vertices=edge_id_by_vertices,
            edges=edges_by_id,
            positions=positions_by_vertex,
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
        output.append(
            RailDecalGeometryFace(
                surface_id=cell.owner_face_id,
                surface_normal=normal,
                vert_keys=[vertex.key for vertex in vertices],
                positions=[Vector(vertex.position) for vertex in vertices],
                u_fracs=[
                    partition.side_sign * vertex.r / alpha
                    for vertex in vertices
                ],
                v_lengths=[vertex.s for vertex in vertices],
                component_kind="RAIL_CORNER",
                component_side=(
                    f"component:{plan.component.component_id}:"
                    f"corner:{partition.partition_id}:face:{cell.owner_face_id}"
                ),
                rail_provenance=provenance,
            )
        )
    _canonicalize_analytic_frontiers(output, plan)
    _apply_canonical_offset_lift(output, offset)
    return tuple(output)


__all__ = (
    "PlanarRailGeometryPlan",
    "RailBoundaryPath",
    "RailBoundaryPiece",
    "RailBoundaryVertex",
    "RailCapTrace",
    "RailAngularChannel",
    "RailAngularPiece",
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
