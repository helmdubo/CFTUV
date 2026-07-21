"""S-DF2-lite: аналитический oracle CornerModel без production routing.

Модуль не строит topology и не выбирает join. Он получает уже derived
polygon CornerModel, задаёт unsigned distance witnesses с явным ``BandSide``
и сравнивает векторные sample-точки финальной materialization с intended
polygon. Raster/SDF texture и min/max-композиции здесь намеренно отсутствуют.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from math import sqrt

from .decal_corner_model import (
    BandSide,
    CornerBoundaryVertex,
    CornerDerivedGeometry,
    CornerModel,
)


_EPS = 1e-10


class DistanceFeatureKind(Enum):
    """Тип аналитического свидетеля ближайшей точки."""

    SOURCE_ORIGIN = "SOURCE_ORIGIN"
    SOURCE_INTERIOR = "SOURCE_INTERIOR"
    CONTOUR_VERTEX = "CONTOUR_VERTEX"
    CONTOUR_EDGE = "CONTOUR_EDGE"


@dataclass(frozen=True)
class DistanceWitness2D:
    """Unsigned distance плюс labeled source/contour witness."""

    distance: float
    closest_point: tuple[float, float]
    gradient: tuple[float, float]
    site_id: object
    side: BandSide
    source_parameter: float
    source_s: float | None
    feature_kind: DistanceFeatureKind

    def __post_init__(self):
        if self.distance < 0.0:
            raise ValueError("DISTANCE_WITNESS_NEGATIVE_DISTANCE")


@dataclass(frozen=True)
class CornerSourceRay2D:
    """Локальная аналитическая source-ray без предположения о rail backend."""

    site_id: int
    origin: tuple[float, float]
    tangent_away: tuple[float, float]
    source_s_origin: float
    source_s_per_chart_unit: float
    side: BandSide

    def __post_init__(self):
        length = _length(self.tangent_away)
        if length <= _EPS:
            raise ValueError("DISTANCE_WITNESS_SOURCE_TANGENT_DEGENERATE")
        object.__setattr__(
            self,
            "tangent_away",
            (
                self.tangent_away[0] / length,
                self.tangent_away[1] / length,
            ),
        )

    def query(self, point):
        """Возвращает unsigned witness к полубесконечной source-ray."""

        delta = _sub2(point, self.origin)
        source_parameter = max(0.0, _dot2(delta, self.tangent_away))
        closest = (
            self.origin[0] + self.tangent_away[0] * source_parameter,
            self.origin[1] + self.tangent_away[1] * source_parameter,
        )
        distance, gradient = _distance_gradient(point, closest)
        return DistanceWitness2D(
            distance=distance,
            closest_point=closest,
            gradient=gradient,
            site_id=self.site_id,
            side=self.side,
            source_parameter=source_parameter,
            source_s=(
                self.source_s_origin
                + source_parameter * self.source_s_per_chart_unit
            ),
            feature_kind=(
                DistanceFeatureKind.SOURCE_ORIGIN
                if source_parameter <= _EPS
                else DistanceFeatureKind.SOURCE_INTERIOR
            ),
        )


@dataclass(frozen=True)
class MaterializedCornerPolygon2D:
    """Одна финальная face, спроецированная в chart oracle."""

    points: tuple[tuple[float, float], ...]
    owner_id: object

    def __post_init__(self):
        if len(self.points) < 3 or abs(_polygon_area2(self.points)) <= _EPS:
            raise ValueError("MATERIALIZED_CORNER_POLYGON_DEGENERATE")


@dataclass(frozen=True)
class CornerFieldSample:
    """Один аналитический sample intended-vs-materialized."""

    point: tuple[float, float]
    intended_material: bool
    materialized_material: bool
    witness: DistanceWitness2D


@dataclass(frozen=True)
class CornerFieldComparison:
    """Полный receipt S-DF2-lite для одного CornerModel."""

    samples: tuple[CornerFieldSample, ...]
    coverage_mismatches: tuple[CornerFieldSample, ...]
    owner_mismatches: tuple[object, ...]
    width_errors: tuple[tuple[str, float, float], ...]
    missing_anchor_ids: tuple[str, ...]

    @property
    def matches(self):
        return not (
            self.coverage_mismatches
            or self.owner_mismatches
            or self.width_errors
            or self.missing_anchor_ids
        )


@dataclass(frozen=True)
class CornerPolygonField2D:
    """Аналитическое intended-поле уже выбранной формы CornerModel.

    Поле не импортирует и не читает join enum. Оно получает ``derived``
    boundary от модели и потому не может независимо пере-вывести форму.
    """

    boundary: tuple[CornerBoundaryVertex, ...]
    side: BandSide
    owner_id: object
    source_rays: tuple[CornerSourceRay2D, CornerSourceRay2D]
    requested_half_width: float

    def __post_init__(self):
        if len(self.boundary) < 3:
            raise ValueError("CORNER_POLYGON_FIELD_BOUNDARY_TOO_SHORT")
        if abs(_polygon_area2(self.points)) <= _EPS:
            raise ValueError("CORNER_POLYGON_FIELD_BOUNDARY_DEGENERATE")
        if self.requested_half_width <= 0.0:
            raise ValueError("CORNER_POLYGON_FIELD_WIDTH_INVALID")

    @classmethod
    def from_model(
        cls,
        model: CornerModel,
        derived: CornerDerivedGeometry,
        *,
        requested_half_width,
    ):
        """Читает готовый derived contour; policy/join здесь не выбирается."""

        if derived.emission_boundary is not derived.collision_boundary:
            raise ValueError("CORNER_POLYGON_FIELD_BOUNDARY_NOT_AUTHORITATIVE")
        source_rays = tuple(
            CornerSourceRay2D(
                site_id=anchor.station_ref.site_id,
                origin=model.seed.apex_ref.chart_point,
                tangent_away=anchor.station_ref.tangent_away,
                source_s_origin=anchor.station_ref.s,
                source_s_per_chart_unit=(
                    anchor.station_ref.source_s_per_chart_unit
                ),
                side=model.seed.side,
            )
            for anchor in (model.p1, model.p2)
        )
        return cls(
            boundary=derived.emission_boundary,
            side=model.seed.side,
            owner_id=(
                "corner-model",
                model.seed.corner_vertex_id,
                model.seed.sector_id,
            ),
            source_rays=source_rays,
            requested_half_width=float(requested_half_width),
        )

    @property
    def points(self):
        return tuple(vertex.chart_point for vertex in self.boundary)

    @property
    def anchors(self):
        by_semantic_id = {
            vertex.semantic_id: vertex for vertex in self.boundary
        }
        try:
            return by_semantic_id["P1"], by_semantic_id["P2"]
        except KeyError as exc:
            raise ValueError("CORNER_POLYGON_FIELD_ANCHOR_MISSING") from exc

    def contains(self, point, *, tolerance=1e-9):
        """Аналитический vector point-in-polygon, включая границу."""

        witness = self.query(point)
        if witness.distance <= tolerance:
            return True
        inside = False
        points = self.points
        for index, first in enumerate(points):
            second = points[(index + 1) % len(points)]
            if (first[1] > point[1]) == (second[1] > point[1]):
                continue
            crossing_x = first[0] + (
                (point[1] - first[1])
                * (second[0] - first[0])
                / (second[1] - first[1])
            )
            if crossing_x > point[0]:
                inside = not inside
        return inside

    def query(self, point):
        """Ближайший labeled witness authoritative polygon boundary."""

        candidates = []
        points = self.points
        for index, first in enumerate(points):
            second = points[(index + 1) % len(points)]
            parameter, closest = _segment_projection(point, first, second)
            distance, gradient = _distance_gradient(point, closest)
            start_id = self.boundary[index].semantic_id
            end_id = self.boundary[(index + 1) % len(points)].semantic_id
            source_s = _boundary_source_s(
                self.boundary[index],
                self.boundary[(index + 1) % len(points)],
                parameter,
            )
            candidates.append(
                DistanceWitness2D(
                    distance=distance,
                    closest_point=closest,
                    gradient=gradient,
                    site_id=(
                        "corner-contour",
                        self.owner_id,
                        start_id,
                        end_id,
                    ),
                    side=self.side,
                    source_parameter=parameter,
                    source_s=source_s,
                    feature_kind=(
                        DistanceFeatureKind.CONTOUR_VERTEX
                        if parameter <= _EPS or parameter >= 1.0 - _EPS
                        else DistanceFeatureKind.CONTOUR_EDGE
                    ),
                )
            )
        return min(
            enumerate(candidates),
            key=lambda item: (item[1].distance, item[0]),
        )[1]

    def query_source(self, site_id, point):
        """DistanceWitness2D к одной из двух incident source-rays."""

        matches = tuple(
            source for source in self.source_rays if source.site_id == site_id
        )
        if len(matches) != 1:
            raise KeyError(f"CORNER_DISTANCE_SOURCE_UNKNOWN: {site_id!r}")
        return matches[0].query(point)

    def compare_materialized(self, polygons, *, tolerance=1e-8):
        """Семплирует финальные vector faces против intended polygon."""

        polygons = tuple(polygons)
        owner_mismatches = tuple(
            polygon.owner_id
            for polygon in polygons
            if polygon.owner_id != self.owner_id
        )
        sample_points = _comparison_sample_points(
            self.points,
            tuple(polygon.points for polygon in polygons),
        )
        samples = tuple(
            CornerFieldSample(
                point=point,
                intended_material=self.contains(point, tolerance=tolerance),
                materialized_material=any(
                    _polygon_contains(polygon.points, point, tolerance)
                    for polygon in polygons
                ),
                witness=self.query(point),
            )
            for point in sample_points
        )
        coverage_mismatches = tuple(
            sample
            for sample in samples
            if sample.intended_material != sample.materialized_material
        )

        width_errors = []
        missing_anchor_ids = []
        materialized_points = tuple(
            point for polygon in polygons for point in polygon.points
        )
        for semantic_id, anchor, source in zip(
            ("P1", "P2"), self.anchors, self.source_rays
        ):
            witness = source.query(anchor.chart_point)
            error = abs(witness.distance - self.requested_half_width)
            if error > tolerance:
                width_errors.append(
                    (
                        semantic_id,
                        witness.distance,
                        self.requested_half_width,
                    )
                )
            if not any(
                _distance(point, anchor.chart_point) <= tolerance
                for point in materialized_points
            ):
                missing_anchor_ids.append(semantic_id)

        return CornerFieldComparison(
            samples=samples,
            coverage_mismatches=coverage_mismatches,
            owner_mismatches=owner_mismatches,
            width_errors=tuple(width_errors),
            missing_anchor_ids=tuple(missing_anchor_ids),
        )


def materialized_corner_polygon(face, *, project=None, owner_id=None):
    """Адаптирует isolated oracle face или финальную DecalGeometryFace."""

    vertices = getattr(face, "vertices", None)
    if vertices is not None and all(
        hasattr(vertex, "chart_point") for vertex in vertices
    ):
        points = tuple(vertex.chart_point for vertex in vertices)
    else:
        positions = getattr(face, "positions", None)
        if positions is None or project is None:
            raise TypeError("MATERIALIZED_CORNER_PROJECTOR_REQUIRED")
        points = tuple(project(position) for position in positions)
    if owner_id is None:
        provenance = getattr(face, "provenance", None)
        owner_id = getattr(provenance, "semantic_owner_id", None)
    return MaterializedCornerPolygon2D(points=points, owner_id=owner_id)


def _sub2(first, second):
    return first[0] - second[0], first[1] - second[1]


def _dot2(first, second):
    return first[0] * second[0] + first[1] * second[1]


def _cross2(first, second):
    return first[0] * second[1] - first[1] * second[0]


def _length(vector):
    return sqrt(_dot2(vector, vector))


def _distance(first, second):
    return _length(_sub2(first, second))


def _distance_gradient(point, closest):
    delta = _sub2(point, closest)
    distance = _length(delta)
    if distance <= _EPS:
        return 0.0, (0.0, 0.0)
    return distance, (delta[0] / distance, delta[1] / distance)


def _segment_projection(point, first, second):
    direction = _sub2(second, first)
    length2 = _dot2(direction, direction)
    if length2 <= _EPS * _EPS:
        return 0.0, first
    parameter = max(
        0.0,
        min(1.0, _dot2(_sub2(point, first), direction) / length2),
    )
    return parameter, (
        first[0] + direction[0] * parameter,
        first[1] + direction[1] * parameter,
    )


def _polygon_area2(points):
    return sum(
        _cross2(points[index], points[(index + 1) % len(points)])
        for index in range(len(points))
    )


def _polygon_contains(points, point, tolerance):
    for index, first in enumerate(points):
        second = points[(index + 1) % len(points)]
        _parameter, closest = _segment_projection(point, first, second)
        if _distance(point, closest) <= tolerance:
            return True
    inside = False
    for index, first in enumerate(points):
        second = points[(index + 1) % len(points)]
        if (first[1] > point[1]) == (second[1] > point[1]):
            continue
        crossing_x = first[0] + (
            (point[1] - first[1])
            * (second[0] - first[0])
            / (second[1] - first[1])
        )
        if crossing_x > point[0]:
            inside = not inside
    return inside


def _mean_point(points):
    return (
        sum(point[0] for point in points) / len(points),
        sum(point[1] for point in points) / len(points),
    )


def _polygon_samples(points):
    samples = list(points)
    samples.append(_mean_point(points))
    for index, first in enumerate(points):
        second = points[(index + 1) % len(points)]
        for parameter in (0.25, 0.5, 0.75):
            samples.append(
                (
                    first[0] + (second[0] - first[0]) * parameter,
                    first[1] + (second[1] - first[1]) * parameter,
                )
            )
    if len(points) > 3:
        for index in range(1, len(points) - 1):
            samples.append(
                _mean_point((points[0], points[index], points[index + 1]))
            )
    return samples


def _comparison_sample_points(intended, materialized):
    points = _polygon_samples(intended)
    for polygon in materialized:
        points.extend(_polygon_samples(polygon))
    quantum = 1e-10
    unique = {}
    for point in points:
        key = (
            round(float(point[0]) / quantum),
            round(float(point[1]) / quantum),
        )
        unique.setdefault(key, (float(point[0]), float(point[1])))
    return tuple(unique[key] for key in sorted(unique))


def _boundary_source_s(first, second, parameter):
    first_values = tuple(ref.s for ref in first.station_refs)
    second_values = tuple(ref.s for ref in second.station_refs)
    first_s = sum(first_values) / len(first_values) if first_values else None
    second_s = (
        sum(second_values) / len(second_values) if second_values else None
    )
    if first_s is None:
        return second_s
    if second_s is None:
        return first_s
    return first_s + (second_s - first_s) * parameter


__all__ = (
    "CornerFieldComparison",
    "CornerFieldSample",
    "CornerPolygonField2D",
    "CornerSourceRay2D",
    "DistanceFeatureKind",
    "DistanceWitness2D",
    "MaterializedCornerPolygon2D",
    "materialized_corner_polygon",
)
