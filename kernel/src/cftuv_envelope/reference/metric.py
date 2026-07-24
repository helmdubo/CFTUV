"""Metric-aware exact operations for planar affine coordinates."""

from __future__ import annotations

from dataclasses import dataclass

import sympy as sp

from ..contracts.analysis import PlanarPatchFrameV1
from ..contracts.metric import (
    AffineChartOrientationV1,
    ExactMatrix2V1,
    ExactRationalV1,
    RationalAffinePlanarMetricV2,
)
from .planar_types import (
    ConstructionCertificate,
    ExactPlanarPoint,
    ExactPlanarVector,
    ExactScalar,
    OrientedSupportLine,
    exact_sign,
    point_add,
    vector_scale,
)


def _scalar(value: ExactRationalV1) -> sp.Rational:
    return sp.Rational(value.numerator, value.denominator)


def _matrix(value: ExactMatrix2V1) -> tuple[tuple[sp.Expr, sp.Expr], ...]:
    return (
        (_scalar(value.m00), _scalar(value.m01)),
        (_scalar(value.m10), _scalar(value.m11)),
    )


@dataclass(frozen=True, slots=True)
class ExactPlanarMetric:
    gram: tuple[tuple[sp.Expr, sp.Expr], tuple[sp.Expr, sp.Expr]]
    inverse_gram: tuple[
        tuple[sp.Expr, sp.Expr], tuple[sp.Expr, sp.Expr]
    ]
    owner_orientation_sign: int

    @classmethod
    def from_descriptor(
        cls,
        descriptor: PlanarPatchFrameV1 | RationalAffinePlanarMetricV2,
    ) -> ExactPlanarMetric:
        if isinstance(descriptor, PlanarPatchFrameV1):
            identity = (
                (sp.Integer(1), sp.Integer(0)),
                (sp.Integer(0), sp.Integer(1)),
            )
            return cls(identity, identity, 1)
        orientation = (
            1
            if descriptor.chart_orientation
            is AffineChartOrientationV1.COORDINATE_CCW_MATCHES_OWNER_PATCH
            else -1
        )
        return cls(
            _matrix(descriptor.exact_gram_matrix),
            _matrix(descriptor.exact_inverse_gram_matrix),
            orientation,
        )

    def dot_g(
        self, left: ExactPlanarVector, right: ExactPlanarVector
    ) -> sp.Expr:
        lx, ly = left.expressions()
        rx, ry = right.expressions()
        gx = self.gram[0][0] * rx + self.gram[0][1] * ry
        gy = self.gram[1][0] * rx + self.gram[1][1] * ry
        return sp.factor(lx * gx + ly * gy)

    def length_g(self, vector: ExactPlanarVector) -> sp.Expr:
        squared = self.dot_g(vector, vector)
        if exact_sign(squared) <= 0:
            raise ValueError("zero vector has no metric length")
        return sp.sqrt(sp.factor(squared))

    def unit_g(self, vector: ExactPlanarVector) -> ExactPlanarVector:
        return vector_scale(vector, 1 / self.length_g(vector))

    def oriented_cross(
        self, left: ExactPlanarVector, right: ExactPlanarVector
    ) -> sp.Expr:
        lx, ly = left.expressions()
        rx, ry = right.expressions()
        return sp.factor(
            self.owner_orientation_sign * (lx * ry - ly * rx)
        )

    def owner_normal_g(
        self,
        tangent: ExactPlanarVector,
        *,
        owner_left: bool,
        normalize: bool = True,
    ) -> ExactPlanarVector:
        tx, ty = tangent.expressions()
        covector_x = self.gram[0][0] * tx + self.gram[0][1] * ty
        covector_y = self.gram[1][0] * tx + self.gram[1][1] * ty
        sign = self.owner_orientation_sign
        normal = ExactPlanarVector.from_values(
            -sign * covector_y,
            sign * covector_x,
        )
        if not owner_left:
            normal = vector_scale(normal, -1)
        return self.unit_g(normal) if normalize else normal

    def distance_to_support_g(
        self,
        point: ExactPlanarPoint,
        support_point: ExactPlanarPoint,
        unit_normal: ExactPlanarVector,
    ) -> sp.Expr:
        px, py = point.expressions()
        sx, sy = support_point.expressions()
        return self.dot_g(
            ExactPlanarVector.from_values(px - sx, py - sy),
            unit_normal,
        )

    def offset_support_g(
        self,
        point: ExactPlanarPoint,
        unit_normal: ExactPlanarVector,
        distance: object,
    ) -> ExactPlanarPoint:
        return point_add(point, vector_scale(unit_normal, distance))

    def support_covector_g(
        self, unit_normal: ExactPlanarVector
    ) -> ExactPlanarVector:
        nx, ny = unit_normal.expressions()
        return ExactPlanarVector.from_values(
            self.gram[0][0] * nx + self.gram[0][1] * ny,
            self.gram[1][0] * nx + self.gram[1][1] * ny,
        )

    def line_through_point_g(
        self,
        support_id: str,
        point: ExactPlanarPoint,
        unit_normal: ExactPlanarVector,
        construction: ConstructionCertificate,
        *,
        normal_speed: object = 1,
    ) -> OrientedSupportLine:
        px, py = point.expressions()
        covector = self.support_covector_g(unit_normal)
        cx, cy = covector.expressions()
        return OrientedSupportLine(
            support_id=support_id,
            normal=covector,
            constant=ExactScalar.from_value(cx * px + cy * py),
            normal_speed=ExactScalar.from_value(normal_speed),
            construction=construction,
        )

    def angle_g(
        self,
        incoming: ExactPlanarVector,
        outgoing: ExactPlanarVector,
    ) -> tuple[sp.Expr, sp.Expr]:
        left = self.unit_g(incoming)
        right = self.unit_g(outgoing)
        cosine = self.dot_g(left, right)
        determinant = sp.factor(
            self.gram[0][0] * self.gram[1][1]
            - self.gram[0][1] * self.gram[1][0]
        )
        sine = sp.factor(
            sp.sqrt(determinant) * self.oriented_cross(left, right)
        )
        return cosine, sine


def dot_G(
    metric: ExactPlanarMetric,
    left: ExactPlanarVector,
    right: ExactPlanarVector,
) -> sp.Expr:
    return metric.dot_g(left, right)


def length_G(
    metric: ExactPlanarMetric,
    vector: ExactPlanarVector,
) -> sp.Expr:
    return metric.length_g(vector)


def unit_G(
    metric: ExactPlanarMetric,
    vector: ExactPlanarVector,
) -> ExactPlanarVector:
    return metric.unit_g(vector)


def owner_normal_G(
    metric: ExactPlanarMetric,
    tangent: ExactPlanarVector,
    *,
    owner_left: bool,
    normalize: bool = True,
) -> ExactPlanarVector:
    return metric.owner_normal_g(
        tangent,
        owner_left=owner_left,
        normalize=normalize,
    )


def distance_to_support_G(
    metric: ExactPlanarMetric,
    point: ExactPlanarPoint,
    support_point: ExactPlanarPoint,
    unit_normal: ExactPlanarVector,
) -> sp.Expr:
    return metric.distance_to_support_g(point, support_point, unit_normal)


def offset_support_G(
    metric: ExactPlanarMetric,
    point: ExactPlanarPoint,
    unit_normal: ExactPlanarVector,
    distance: object,
) -> ExactPlanarPoint:
    return metric.offset_support_g(point, unit_normal, distance)


def angle_G(
    metric: ExactPlanarMetric,
    incoming: ExactPlanarVector,
    outgoing: ExactPlanarVector,
) -> tuple[sp.Expr, sp.Expr]:
    return metric.angle_g(incoming, outgoing)
