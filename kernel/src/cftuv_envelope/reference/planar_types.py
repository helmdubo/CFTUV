"""Точные Blender-free примитивы planar reference evaluator.

Координаты хранятся как канонические SymPy expressions.  SymPy здесь служит
алгебраическим числовым backend, но не Boolean/arrangement backend: разбиение
сегментов, история кривых и union принадлежат этому пакету.
"""

from __future__ import annotations

from dataclasses import dataclass
from decimal import Decimal
from enum import Enum
from fractions import Fraction
from functools import cmp_to_key, lru_cache
from typing import Iterable

import sympy as sp


class CertifiedPredicateUndecidable(ValueError):
    """Exact sign/equality could not be proven; tolerance fallback is forbidden."""


def _expr(value: ExactScalar | sp.Expr | Decimal | Fraction | int | float | str) -> sp.Expr:
    if isinstance(value, ExactScalar):
        return value.as_expr()
    if isinstance(value, sp.Expr):
        return value
    if isinstance(value, Decimal):
        return sp.Rational(str(value))
    if isinstance(value, Fraction):
        return sp.Rational(value.numerator, value.denominator)
    if isinstance(value, float):
        return sp.Rational(str(value))
    if isinstance(value, int):
        return sp.Integer(value)
    return sp.sympify(value)


def _canonical_expr(value: sp.Expr) -> str:
    simplified = sp.factor(sp.cancel(value))
    return sp.srepr(simplified)


@lru_cache(maxsize=32768)
def _parse_expr(expression: str) -> sp.Expr:
    return sp.sympify(expression)


@dataclass(frozen=True, slots=True)
class ExactScalar:
    expression: str

    @classmethod
    def from_value(
        cls, value: ExactScalar | sp.Expr | Decimal | Fraction | int | float | str
    ) -> ExactScalar:
        if isinstance(value, cls):
            return value
        return cls(_canonical_expr(_expr(value)))

    def as_expr(self) -> sp.Expr:
        return _parse_expr(self.expression)


def exact_sign(value: ExactScalar | sp.Expr | Decimal | Fraction | int | float | str) -> int:
    candidate = sp.factor(sp.cancel(_expr(value)))
    if candidate == 0 or candidate.is_zero is True:
        return 0
    if candidate.is_positive is True:
        return 1
    if candidate.is_negative is True:
        return -1
    equals_zero = candidate.equals(0)
    if equals_zero is True:
        return 0
    positive = sp.ask(sp.Q.positive(candidate))
    negative = sp.ask(sp.Q.negative(candidate))
    if positive is True:
        return 1
    if negative is True:
        return -1
    raise CertifiedPredicateUndecidable(f"cannot prove sign of {sp.srepr(candidate)}")


def exact_equal(left: ExactScalar | sp.Expr, right: ExactScalar | sp.Expr) -> bool:
    return exact_sign(_expr(left) - _expr(right)) == 0


@dataclass(frozen=True, slots=True)
class ExactPlanarPoint:
    x: ExactScalar
    y: ExactScalar

    @classmethod
    def from_values(cls, x: object, y: object) -> ExactPlanarPoint:
        return cls(ExactScalar.from_value(x), ExactScalar.from_value(y))

    def expressions(self) -> tuple[sp.Expr, sp.Expr]:
        return self.x.as_expr(), self.y.as_expr()


@dataclass(frozen=True, slots=True)
class ExactPlanarVector:
    x: ExactScalar
    y: ExactScalar

    @classmethod
    def from_values(cls, x: object, y: object) -> ExactPlanarVector:
        return cls(ExactScalar.from_value(x), ExactScalar.from_value(y))

    def expressions(self) -> tuple[sp.Expr, sp.Expr]:
        return self.x.as_expr(), self.y.as_expr()


def point_add(point: ExactPlanarPoint, vector: ExactPlanarVector) -> ExactPlanarPoint:
    px, py = point.expressions()
    vx, vy = vector.expressions()
    return ExactPlanarPoint.from_values(px + vx, py + vy)


def point_sub(left: ExactPlanarPoint, right: ExactPlanarPoint) -> ExactPlanarVector:
    lx, ly = left.expressions()
    rx, ry = right.expressions()
    return ExactPlanarVector.from_values(lx - rx, ly - ry)


def vector_scale(vector: ExactPlanarVector, scalar: object) -> ExactPlanarVector:
    vx, vy = vector.expressions()
    factor = _expr(scalar)
    return ExactPlanarVector.from_values(vx * factor, vy * factor)


def dot(left: ExactPlanarVector, right: ExactPlanarVector) -> sp.Expr:
    lx, ly = left.expressions()
    rx, ry = right.expressions()
    return sp.factor(lx * rx + ly * ry)


def cross(left: ExactPlanarVector, right: ExactPlanarVector) -> sp.Expr:
    lx, ly = left.expressions()
    rx, ry = right.expressions()
    return sp.factor(lx * ry - ly * rx)


def squared_length(vector: ExactPlanarVector) -> sp.Expr:
    return dot(vector, vector)


def unit(vector: ExactPlanarVector) -> ExactPlanarVector:
    length_squared = squared_length(vector)
    if exact_sign(length_squared) <= 0:
        raise ValueError("zero planar vector has no unit direction")
    return vector_scale(vector, 1 / sp.sqrt(length_squared))


def left_normal(vector: ExactPlanarVector) -> ExactPlanarVector:
    x, y = vector.expressions()
    return ExactPlanarVector.from_values(-y, x)


def right_normal(vector: ExactPlanarVector) -> ExactPlanarVector:
    x, y = vector.expressions()
    return ExactPlanarVector.from_values(y, -x)


class ConstructionKind(str, Enum):
    SOURCE_VERTEX = "SOURCE_VERTEX"
    SUPPORT_SUPPORT_INTERSECTION = "SUPPORT_SUPPORT_INTERSECTION"
    SUPPORT_BOUNDARY_INTERSECTION = "SUPPORT_BOUNDARY_INTERSECTION"
    BOUNDARY_BOUNDARY_INTERSECTION = "BOUNDARY_BOUNDARY_INTERSECTION"
    EVENT_ANCHOR = "EVENT_ANCHOR"


@dataclass(frozen=True, slots=True)
class ConstructionCertificate:
    kind: ConstructionKind
    source_vertex_ids: frozenset[str] = frozenset()
    support_ids: frozenset[str] = frozenset()
    boundary_constraint_ids: frozenset[str] = frozenset()
    physical_edge_ids: frozenset[str] = frozenset()
    event_key: str | None = None
    generator_segment_ids: frozenset[str] = frozenset()


@dataclass(frozen=True, slots=True)
class OrientedSupportLine:
    support_id: str
    normal: ExactPlanarVector
    constant: ExactScalar
    normal_speed: ExactScalar
    construction: ConstructionCertificate

    def at_alpha(self, alpha: object) -> OrientedSupportLine:
        return OrientedSupportLine(
            support_id=self.support_id,
            normal=self.normal,
            constant=ExactScalar.from_value(
                self.constant.as_expr() + self.normal_speed.as_expr() * _expr(alpha)
            ),
            normal_speed=self.normal_speed,
            construction=self.construction,
        )


@dataclass(frozen=True, slots=True)
class BoundedSupportSegment:
    segment_id: str
    start: ExactPlanarPoint
    end: ExactPlanarPoint
    support_ids: frozenset[str]
    boundary_constraint_ids: frozenset[str]
    provenance: object
    start_constructions: frozenset[ConstructionCertificate]
    end_constructions: frozenset[ConstructionCertificate]

    def reversed(self) -> BoundedSupportSegment:
        return BoundedSupportSegment(
            segment_id=self.segment_id,
            start=self.end,
            end=self.start,
            support_ids=self.support_ids,
            boundary_constraint_ids=self.boundary_constraint_ids,
            provenance=self.provenance,
            start_constructions=self.end_constructions,
            end_constructions=self.start_constructions,
        )


@dataclass(frozen=True, slots=True)
class PlanarLoop:
    loop_id: str
    segments: tuple[BoundedSupportSegment, ...]

    @property
    def points(self) -> tuple[ExactPlanarPoint, ...]:
        return tuple(segment.start for segment in self.segments)


@dataclass(frozen=True, slots=True)
class PlanarRegion:
    region_id: str
    outer: PlanarLoop
    holes: tuple[PlanarLoop, ...] = ()
    contributor_instance_ids: frozenset[str] = frozenset()
    contributor_spec_ids: frozenset[str] = frozenset()


def line_through_point(
    support_id: str,
    point: ExactPlanarPoint,
    normal: ExactPlanarVector,
    construction: ConstructionCertificate,
    *,
    normal_speed: object = 1,
) -> OrientedSupportLine:
    px, py = point.expressions()
    nx, ny = normal.expressions()
    return OrientedSupportLine(
        support_id=support_id,
        normal=normal,
        constant=ExactScalar.from_value(nx * px + ny * py),
        normal_speed=ExactScalar.from_value(normal_speed),
        construction=construction,
    )


def support_intersection(
    left: OrientedSupportLine, right: OrientedSupportLine
) -> ExactPlanarPoint:
    a, b = left.normal.expressions()
    c, d = right.normal.expressions()
    e = left.constant.as_expr()
    f = right.constant.as_expr()
    determinant = sp.factor(a * d - b * c)
    if exact_sign(determinant) == 0:
        raise ValueError("parallel supports have no unique intersection")
    return ExactPlanarPoint.from_values(
        (e * d - b * f) / determinant,
        (a * f - e * c) / determinant,
    )


def point_key(point: ExactPlanarPoint) -> tuple[str, str]:
    return point.x.expression, point.y.expression


def points_equal(left: ExactPlanarPoint, right: ExactPlanarPoint) -> bool:
    lx, ly = left.expressions()
    rx, ry = right.expressions()
    return exact_sign(lx - rx) == 0 and exact_sign(ly - ry) == 0


def polygon_signed_area(points: Iterable[ExactPlanarPoint]) -> sp.Expr:
    vertices = tuple(points)
    if len(vertices) < 3:
        return sp.Integer(0)
    twice_area = sp.Integer(0)
    for start, end in zip(vertices, vertices[1:] + vertices[:1]):
        sx, sy = start.expressions()
        ex, ey = end.expressions()
        twice_area += sx * ey - sy * ex
    return sp.factor(twice_area / 2)


def _half(vector: ExactPlanarVector) -> int:
    x, y = vector.expressions()
    sy = exact_sign(y)
    sx = exact_sign(x)
    return 0 if sy > 0 or (sy == 0 and sx >= 0) else 1


def compare_directions(left: ExactPlanarVector, right: ExactPlanarVector) -> int:
    left_half = _half(left)
    right_half = _half(right)
    if left_half != right_half:
        return -1 if left_half < right_half else 1
    turn = exact_sign(cross(left, right))
    if turn:
        return -1 if turn > 0 else 1
    return 0


def sort_points_around(
    center: ExactPlanarPoint, points: Iterable[ExactPlanarPoint]
) -> tuple[ExactPlanarPoint, ...]:
    return tuple(
        sorted(
            points,
            key=cmp_to_key(
                lambda left, right: compare_directions(
                    point_sub(left, center), point_sub(right, center)
                )
            ),
        )
    )
