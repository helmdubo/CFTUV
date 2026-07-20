"""Фазированный контракт локальной геометрии decal-угла.

Модуль не знает о Voronoi, arrangement, BMesh и способе lift. Его вход —
compile-static ``CornerSeed`` и два якоря, уже прочитанные из локально
клипнутых собственных strip-сегментов. Поэтому collision, ownership, UV,
projection и emission получают одну и ту же границу, а не восстанавливают
форму угла независимо.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from math import isfinite, sqrt

from .model import CornerJoinMode


_EPS = 1e-9


class BandSide(str, Enum):
    """Знаковая сторона ленты в нейтральной поперечной координате ``r``."""

    NEGATIVE = "NEGATIVE"
    POSITIVE = "POSITIVE"


class CornerAnchorKind(str, Enum):
    """Три и только три семантических якоря модели."""

    V = "V"
    P1 = "P1"
    P2 = "P2"


@dataclass(frozen=True)
class CornerStationRef:
    """Нейтральная station-ссылка собственного strip-маршрута."""

    route_id: object
    site_id: int
    source_edge_id: int
    station_key: object
    s: float
    r: float
    tangent_away: tuple[float, float]

    def __post_init__(self):
        if len(self.tangent_away) != 2:
            raise ValueError("CORNER_STATION_TANGENT_INVALID")
        if not all(
            isfinite(float(value))
            for value in (*self.tangent_away, self.s, self.r)
        ):
            raise ValueError("CORNER_STATION_NON_FINITE")
        length = sqrt(
            float(self.tangent_away[0]) ** 2
            + float(self.tangent_away[1]) ** 2
        )
        if length <= _EPS:
            raise ValueError("CORNER_STATION_TANGENT_DEGENERATE")


@dataclass(frozen=True)
class CornerPointProvenance:
    """Source-факты точки до глобальной конкуренции."""

    source_face_id: object
    source_edge_ids: tuple[int, ...]
    route_ids: tuple[object, ...]
    station_keys: tuple[object, ...]
    domain_location: object | None = None

    def __post_init__(self):
        if not self.source_edge_ids:
            raise ValueError("CORNER_POINT_SOURCE_EDGE_MISSING")
        if not self.route_ids:
            raise ValueError("CORNER_POINT_ROUTE_MISSING")
        if not self.station_keys:
            raise ValueError("CORNER_POINT_STATION_MISSING")


@dataclass(frozen=True)
class CornerVertexRef:
    """Фактический V после projection/offset, а не только source id."""

    key: object
    chart_point: tuple[float, float]
    position: tuple[float, float, float]
    provenance: CornerPointProvenance


@dataclass(frozen=True)
class CornerAnchor:
    """P1/P2, взятый из вершины локально клипнутого own-strip."""

    kind: CornerAnchorKind
    key: object
    chart_point: tuple[float, float]
    provenance: CornerPointProvenance
    station_ref: CornerStationRef

    def __post_init__(self):
        if self.kind not in {CornerAnchorKind.P1, CornerAnchorKind.P2}:
            raise ValueError("CORNER_OUTER_ANCHOR_KIND_INVALID")
        if self.station_ref.source_edge_id not in self.provenance.source_edge_ids:
            raise ValueError("CORNER_ANCHOR_EDGE_PROVENANCE_DESYNC")
        if self.station_ref.route_id not in self.provenance.route_ids:
            raise ValueError("CORNER_ANCHOR_ROUTE_PROVENANCE_DESYNC")
        if self.station_ref.station_key not in self.provenance.station_keys:
            raise ValueError("CORNER_ANCHOR_STATION_PROVENANCE_DESYNC")


@dataclass(frozen=True)
class CornerStripVertex:
    """Вершина owner-domain/own-strip локального клипа."""

    key: object
    chart_point: tuple[float, float]
    provenance: CornerPointProvenance
    station_ref: CornerStationRef


@dataclass(frozen=True)
class LocalClippedCornerStrip:
    """Клип одного incident strip до inter-corner competition."""

    site_id: int
    vertices: tuple[CornerStripVertex, ...]
    outer_corner_key: object

    @property
    def outer_corner(self):
        matches = tuple(
            vertex
            for vertex in self.vertices
            if vertex.key == self.outer_corner_key
        )
        if len(matches) != 1:
            raise ValueError(
                "CORNER_LOCAL_STRIP_OUTER_KEY_INVALID: "
                f"site={self.site_id} key={self.outer_corner_key!r}"
            )
        return matches[0]


@dataclass(frozen=True)
class CornerSeed:
    """Compile-static факты угла до width-dependent локального клипа."""

    apex_ref: CornerVertexRef
    corner_vertex_id: int
    incident_site_ids: tuple[int, int]
    side: BandSide
    sector_id: int | None
    owner_surface_id: int
    join: CornerJoinMode

    def __post_init__(self):
        if len(self.incident_site_ids) != 2:
            raise ValueError("CORNER_SEED_INCIDENT_PAIR_REQUIRED")
        if self.incident_site_ids[0] == self.incident_site_ids[1]:
            raise ValueError("CORNER_SEED_INCIDENT_PAIR_DEGENERATE")
        object.__setattr__(self, "join", CornerJoinMode(self.join))


@dataclass(frozen=True)
class CornerBoundaryVertex:
    """Одна вершина authoritative material boundary."""

    semantic_id: str
    key: object
    chart_point: tuple[float, float]
    provenance: CornerPointProvenance
    station_refs: tuple[CornerStationRef, ...]


@dataclass(frozen=True)
class CornerBoundaryTrace:
    """Родословная materialized vertex на одном ребре semantic contour."""

    vertex_key: object
    contour_edge: tuple[CornerAnchorKind, CornerAnchorKind]
    parameter: float


@dataclass(frozen=True)
class CornerDerivedGeometry:
    """Одна граница, разделяемая всеми локальными потребителями."""

    collision_boundary: tuple[CornerBoundaryVertex, ...]
    ownership_boundary: tuple[CornerBoundaryVertex, ...]
    uv_boundary: tuple[CornerBoundaryVertex, ...]
    projection_boundary: tuple[CornerBoundaryVertex, ...]
    emission_boundary: tuple[CornerBoundaryVertex, ...]

    def __post_init__(self):
        boundary = self.collision_boundary
        if not (
            self.ownership_boundary is boundary
            and self.uv_boundary is boundary
            and self.projection_boundary is boundary
            and self.emission_boundary is boundary
        ):
            raise ValueError("CORNER_DERIVED_BOUNDARY_NOT_SHARED")


def _cross2(first, second):
    return first[0] * second[1] - first[1] * second[0]


def _sub2(first, second):
    return first[0] - second[0], first[1] - second[1]


def _distance2(first, second):
    delta = _sub2(first, second)
    return sqrt(delta[0] * delta[0] + delta[1] * delta[1])


def _line_intersection(point_a, direction_a, point_b, direction_b):
    denominator = _cross2(direction_a, direction_b)
    if abs(denominator) <= _EPS:
        return None
    factor = _cross2(_sub2(point_b, point_a), direction_b) / denominator
    return (
        point_a[0] + direction_a[0] * factor,
        point_a[1] + direction_a[1] * factor,
    )


def _point_segment_parameter(point, first, second):
    direction = _sub2(second, first)
    length2 = direction[0] * direction[0] + direction[1] * direction[1]
    if length2 <= _EPS * _EPS:
        return None
    parameter = (
        (point[0] - first[0]) * direction[0]
        + (point[1] - first[1]) * direction[1]
    ) / length2
    projection = (
        first[0] + direction[0] * parameter,
        first[1] + direction[1] * parameter,
    )
    return parameter, _distance2(point, projection)


@dataclass(frozen=True)
class CornerModel:
    """Локальная форма после own-strip clip и до global competition."""

    seed: CornerSeed
    p1: CornerAnchor
    p2: CornerAnchor

    @classmethod
    def from_local_strips(cls, seed, first_strip, second_strip):
        """Читает P1/P2 из фактических outer corners двух own-strips."""

        strips = (first_strip, second_strip)
        if tuple(strip.site_id for strip in strips) != seed.incident_site_ids:
            raise ValueError("CORNER_LOCAL_STRIP_ORDER_DESYNC")
        vertices = tuple(strip.outer_corner for strip in strips)
        anchors = tuple(
            CornerAnchor(
                kind=kind,
                key=vertex.key,
                chart_point=vertex.chart_point,
                provenance=vertex.provenance,
                station_ref=vertex.station_ref,
            )
            for kind, vertex in zip(
                (CornerAnchorKind.P1, CornerAnchorKind.P2), vertices
            )
        )
        return cls(seed=seed, p1=anchors[0], p2=anchors[1])

    def __post_init__(self):
        if self.p1.kind is not CornerAnchorKind.P1:
            raise ValueError("CORNER_MODEL_P1_KIND_INVALID")
        if self.p2.kind is not CornerAnchorKind.P2:
            raise ValueError("CORNER_MODEL_P2_KIND_INVALID")
        anchor_edges = (
            self.p1.station_ref.site_id,
            self.p2.station_ref.site_id,
        )
        if anchor_edges != self.seed.incident_site_ids:
            raise ValueError(
                "CORNER_MODEL_ORDERED_SITE_DESYNC: "
                f"seed={self.seed.incident_site_ids!r} anchors={anchor_edges!r}"
            )
        if self.p1 is self.p2 or self.p1.key == self.p2.key:
            raise ValueError("CORNER_MODEL_ANCHORS_DEGENERATE")

    @property
    def semantic_anchors(self):
        """Authoritative V/P1/P2 независимо от material subdivision."""

        return self.seed.apex_ref, self.p1, self.p2

    def _anchor_vertex(self, anchor):
        if isinstance(anchor, CornerVertexRef):
            return CornerBoundaryVertex(
                semantic_id=CornerAnchorKind.V.value,
                key=anchor.key,
                chart_point=anchor.chart_point,
                provenance=anchor.provenance,
                station_refs=(),
            )
        return CornerBoundaryVertex(
            semantic_id=anchor.kind.value,
            key=anchor.key,
            chart_point=anchor.chart_point,
            provenance=anchor.provenance,
            station_refs=(anchor.station_ref,),
        )

    def _material_boundary(self, apex_limit=None):
        vertex_v = self._anchor_vertex(self.seed.apex_ref)
        vertex_p1 = self._anchor_vertex(self.p1)
        vertex_p2 = self._anchor_vertex(self.p2)

        # Единственная ветка формы по join во всём runtime-контракте.
        if self.seed.join is CornerJoinMode.BEVEL:
            return (vertex_v, vertex_p1, vertex_p2)

        intersection = _line_intersection(
            self.p1.chart_point,
            self.p1.station_ref.tangent_away,
            self.p2.chart_point,
            self.p2.station_ref.tangent_away,
        )
        if intersection is None or (
            apex_limit is not None
            and _distance2(self.seed.apex_ref.chart_point, intersection)
            > float(apex_limit)
        ):
            return (vertex_v, vertex_p1, vertex_p2)
        provenance = CornerPointProvenance(
            source_face_id=self.p1.provenance.source_face_id,
            source_edge_ids=tuple(
                dict.fromkeys(
                    self.p1.provenance.source_edge_ids
                    + self.p2.provenance.source_edge_ids
                )
            ),
            route_ids=tuple(
                dict.fromkeys(
                    self.p1.provenance.route_ids
                    + self.p2.provenance.route_ids
                )
            ),
            station_keys=tuple(
                dict.fromkeys(
                    self.p1.provenance.station_keys
                    + self.p2.provenance.station_keys
                )
            ),
            domain_location=None,
        )
        miter = CornerBoundaryVertex(
            semantic_id="MITER_APEX",
            key=(
                "corner-miter-apex",
                self.seed.owner_surface_id,
                self.seed.corner_vertex_id,
                self.seed.sector_id,
            ),
            chart_point=intersection,
            provenance=provenance,
            station_refs=(self.p1.station_ref, self.p2.station_ref),
        )
        return (vertex_v, vertex_p1, miter, vertex_p2)

    def derive(self, *, apex_limit=None):
        """Возвращает один shared boundary для всех локальных стадий."""

        boundary = self._material_boundary(apex_limit=apex_limit)
        return CornerDerivedGeometry(
            collision_boundary=boundary,
            ownership_boundary=boundary,
            uv_boundary=boundary,
            projection_boundary=boundary,
            emission_boundary=boundary,
        )

    def trace_materialized_vertex(self, vertex_key, point, *, tolerance=1e-8):
        """Доказывает принадлежность точки одному из рёбер V/P1/P2."""

        anchors = (
            (CornerAnchorKind.V, self.seed.apex_ref.chart_point),
            (CornerAnchorKind.P1, self.p1.chart_point),
            (CornerAnchorKind.P2, self.p2.chart_point),
        )
        candidates = []
        for index, (start_kind, start) in enumerate(anchors):
            end_kind, end = anchors[(index + 1) % len(anchors)]
            result = _point_segment_parameter(point, start, end)
            if result is None:
                continue
            parameter, distance = result
            if (
                distance <= tolerance
                and -tolerance <= parameter <= 1.0 + tolerance
            ):
                candidates.append(
                    (
                        distance,
                        index,
                        CornerBoundaryTrace(
                            vertex_key=vertex_key,
                            contour_edge=(start_kind, end_kind),
                            parameter=max(0.0, min(1.0, parameter)),
                        ),
                    )
                )
        if not candidates:
            raise ValueError(
                "CORNER_MATERIAL_VERTEX_OUTSIDE_SEMANTIC_CONTOUR: "
                f"key={vertex_key!r} point={point!r}"
            )
        return min(candidates, key=lambda item: (item[0], item[1]))[2]

    def resolve(self, materialized_vertices, *, tolerance=1e-8):
        """Создаёт post-competition view без повторного вывода формы."""

        derived = self.derive()
        traces = tuple(
            self.trace_materialized_vertex(key, point, tolerance=tolerance)
            for key, point in materialized_vertices
        )
        return ResolvedCornerView(
            model=self,
            p1=self.p1,
            p2=self.p2,
            derived=derived,
            materialized_vertices=tuple(materialized_vertices),
            traces=traces,
        )


@dataclass(frozen=True)
class ResolvedCornerView:
    """Post-competition view, сохраняющий identity исходных P1/P2."""

    model: CornerModel
    p1: CornerAnchor
    p2: CornerAnchor
    derived: CornerDerivedGeometry
    materialized_vertices: tuple[tuple[object, tuple[float, float]], ...]
    traces: tuple[CornerBoundaryTrace, ...]

    def __post_init__(self):
        if self.p1 is not self.model.p1 or self.p2 is not self.model.p2:
            raise ValueError("RESOLVED_CORNER_REDERIVED_ANCHOR")
        if len(self.materialized_vertices) != len(self.traces):
            raise ValueError("RESOLVED_CORNER_TRACE_COUNT_MISMATCH")


__all__ = (
    "BandSide",
    "CornerAnchor",
    "CornerAnchorKind",
    "CornerBoundaryTrace",
    "CornerBoundaryVertex",
    "CornerDerivedGeometry",
    "CornerModel",
    "CornerPointProvenance",
    "CornerSeed",
    "CornerStationRef",
    "CornerStripVertex",
    "CornerVertexRef",
    "LocalClippedCornerStrip",
    "ResolvedCornerView",
)
